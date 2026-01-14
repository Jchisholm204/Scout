/**
 * @file usb_interface.c
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2025-10-17
 * @modified Last Modified: 2025-10-17
 *
 * @copyright Copyright (c) 2025
 */

#include "usb/usb_interface.h"

#include "FreeRTOS.h"
#include "config/FreeRTOSConfig.h"
#include "config/nvic.h"
#include "config/pin_cfg.h"
#include "drivers/stusb/usb.h"
#include "hal/hal_usb.h"
#include "os/systime.h"
#include "stm32f446xx.h"
#include "task.h"
#include "usb/usb_desc.h"
#include "usb_cb_defs.h"
#include "usb_packet.h"

#include <stdio.h>

// USB Device
usbd_device udev;
uint32_t usb0_buf[CDC_EP0_SIZE]; // EP0 Buffer
// Lidar Data transmission interleaving
volatile enum eCBLidar lidar_primary_rx = eLidarFront;

// USBD Configuration Callback
static usbd_respond udev_setconf(usbd_device *dev, uint8_t cfg);

// External Interfacing
struct usbi usbi = {0};
StaticQueue_t usbi_lidar_rx_sqh;
struct udev_pkt_lidar usbi_lidar_rx_buf[USBI_LIDAR_BUF_SIZE] = {0};
StaticQueue_t usbi_lidar_tx_sqh = {0};
struct udev_pkt_lidar usbi_lidar_tx_buf[USBI_LIDAR_BUF_SIZE] = {0};
StaticQueue_t usbi_ctrl_tx_sqh = {0};
struct udev_pkt_ctrl_tx usbi_ctrl_tx_buf[USBI_CTRL_BUF_SIZE] = {0};
StaticQueue_t usbi_ctrl_rx_sqh = {0};
struct udev_pkt_ctrl_rx usbi_ctrl_rx_buf[USBI_CTRL_BUF_SIZE] = {0};

struct usbi *usbi_init(void) {
    // Initialize the USB Device
    // GPIO Initialization
    hal_usb_init_rcc();
    // libusb_stm32 init device
    usbd_init(&udev, &usbd_hw, CDC_EP0_SIZE, usb0_buf, sizeof(usb0_buf));
    // Apply the device registration function
    usbd_reg_config(&udev, udev_setconf);
    // Apply the USBD Descriptors
    usbd_reg_control(&udev, udev_control);
    usbd_reg_descr(&udev, udev_getdesc);

    // External Interface Setup

    usbi.lidar_rx = xQueueCreateStatic(USBI_LIDAR_BUF_SIZE,
                                       sizeof(struct udev_pkt_lidar),
                                       (uint8_t *) usbi_lidar_rx_buf,
                                       &usbi_lidar_rx_sqh);

    usbi.lidar_tx = xQueueCreateStatic(USBI_LIDAR_BUF_SIZE,
                                       sizeof(struct udev_pkt_lidar),
                                       (uint8_t *) usbi_lidar_tx_buf,
                                       &usbi_lidar_tx_sqh);

    usbi.ctrl_tx = xQueueCreateStatic(USBI_CTRL_BUF_SIZE,
                                      sizeof(struct udev_pkt_ctrl_tx),
                                      (uint8_t *) usbi_ctrl_tx_buf,
                                      &usbi_ctrl_tx_sqh);

    usbi.ctrl_rx = xQueueCreateStatic(USBI_CTRL_BUF_SIZE,
                                      sizeof(struct udev_pkt_ctrl_rx),
                                      (uint8_t *) usbi_ctrl_rx_buf,
                                      &usbi_ctrl_rx_sqh);

    // Enable USB OTG Interrupt
    NVIC_SetPriority(OTG_FS_IRQn, 8);
    NVIC_EnableIRQ(OTG_FS_IRQn);
    // Enable the USB Device
    usbd_enable(&udev, 1);
    usbd_connect(&udev, 1);

    return &usbi;
}

volatile int queue_fails = 0;
// USBD RX/TX Callbacks: Control COM Port
// Triggered during Virtual Communications Interface events
static void ctrl_rxtx(usbd_device *dev, uint8_t evt, uint8_t ep) {
    BaseType_t higher_woken = pdFALSE;
    if (evt == usbd_evt_eprx) {
        struct udev_pkt_ctrl_tx pkt_ctrl_tx = {0};
        int len = usbd_ep_read(
            dev, ep, (void *) &pkt_ctrl_tx, sizeof(struct udev_pkt_ctrl_tx));
        if (!usbi.lidar_rx)
            return;
        BaseType_t r =
            xQueueOverwriteFromISR(usbi.ctrl_tx, &pkt_ctrl_tx, &higher_woken);
        if (r != pdTRUE) {
            queue_fails++;
        }
    } else {
        struct udev_pkt_ctrl_rx pkt_ctrl_rx = {0};
        // char ar[] = "Hello1\nHello2\nHello3\nHello4\n";
        if (xQueueReceiveFromISR(usbi.ctrl_rx, &pkt_ctrl_rx, &higher_woken) ==
            pdTRUE) {
            usbd_ep_write(dev,
                          ep,
                          (void *) &pkt_ctrl_rx,
                          sizeof(struct udev_pkt_ctrl_rx));
        } else {
            usbd_ep_write(dev, ep, (void *) &pkt_ctrl_rx, 0);
        }
    }
    portYIELD_FROM_ISR(higher_woken);
}
static volatile struct udev_pkt_lidar pkt_lidar __attribute__((aligned)) = {0};
static void lidar_rxtx(usbd_device *dev, uint8_t evt, uint8_t ep) {
    BaseType_t higher_woken = pdFALSE;
    if (evt == usbd_evt_eprx) {
        usbd_ep_read(
            dev, ep, (void *) &pkt_lidar, sizeof(struct udev_pkt_lidar));
        // Dont worry about dropping packets
        (void) xQueueSendToBackFromISR(usbi.lidar_rx,
                                       &pkt_lidar,
                                       &higher_woken);
    } else {
        if (xQueueReceiveFromISR(usbi.lidar_tx, &pkt_lidar, &higher_woken) ==
            pdTRUE) {
            usbd_ep_write(
                dev, ep, (void *) &pkt_lidar, sizeof(struct udev_pkt_lidar));
        } else {
            usbd_ep_write(dev, ep, (void *) 0, 0);
        }
    }
    portYIELD_FROM_ISR(higher_woken);
}

static usbd_respond udev_setconf(usbd_device *dev, uint8_t cfg) {
    switch (cfg) {
    case 0:
        /* deconfiguring device */
        usbd_ep_deconfig(dev, CTRL_NTF_EP);
        usbd_ep_deconfig(dev, CTRL_TXD_EP);
        usbd_ep_deconfig(dev, CTRL_RXD_EP);
        usbd_reg_endpoint(dev, CTRL_RXD_EP, 0);
        usbd_reg_endpoint(dev, CTRL_TXD_EP, 0);

        usbd_ep_deconfig(dev, LIDAR_NTF_EP);
        usbd_ep_deconfig(dev, LIDAR_TXD_EP);
        usbd_ep_deconfig(dev, LIDAR_RXD_EP);
        usbd_reg_endpoint(dev, LIDAR_RXD_EP, 0);
        usbd_reg_endpoint(dev, LIDAR_TXD_EP, 0);
        return usbd_ack;
    case 1:
        /* configuring device */
        usbd_ep_config(dev,
                       CTRL_RXD_EP,
                       USB_EPTYPE_BULK /*| USB_EPTYPE_DBLBUF*/,
                       CTRL_DATA_SZ);
        usbd_ep_config(dev,
                       CTRL_TXD_EP,
                       USB_EPTYPE_BULK /*| USB_EPTYPE_DBLBUF*/,
                       CTRL_DATA_SZ);
        usbd_ep_config(dev, CTRL_NTF_EP, USB_EPTYPE_INTERRUPT, CTRL_NTF_SZ);

        usbd_ep_config(dev,
                       LIDAR_RXD_EP,
                       USB_EPTYPE_BULK /*| USB_EPTYPE_DBLBUF*/,
                       LIDAR_DATA_SZ);
        usbd_ep_config(dev,
                       LIDAR_TXD_EP,
                       USB_EPTYPE_BULK /*| USB_EPTYPE_DBLBUF*/,
                       LIDAR_DATA_SZ);
        usbd_ep_config(dev, LIDAR_NTF_EP, USB_EPTYPE_INTERRUPT, CTRL_NTF_SZ);

        usbd_reg_endpoint(dev, CTRL_RXD_EP, ctrl_rxtx);
        usbd_reg_endpoint(dev, CTRL_TXD_EP, ctrl_rxtx);
        usbd_reg_endpoint(dev, LIDAR_RXD_EP, lidar_rxtx);
        usbd_reg_endpoint(dev, LIDAR_TXD_EP, lidar_rxtx);

        usbd_ep_write(dev, CTRL_TXD_EP, 0, 0);
        usbd_ep_write(dev, CTRL_TXD_EP, 0, 0);
        usbd_ep_write(dev, LIDAR_TXD_EP, 0, 0);
        usbd_ep_write(dev, LIDAR_TXD_EP, 0, 0);
        return usbd_ack;
    default:
        return usbd_fail;
    }
}

// Interrupt handler for USB OTG FS
void OTG_FS_IRQHandler(void) {
    usbd_poll(&udev);
}
