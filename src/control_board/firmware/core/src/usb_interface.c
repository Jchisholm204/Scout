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
static usbd_device udev;
static uint32_t usb0_buf[CDC_EP0_SIZE]; // EP0 Buffer
// Lidar Data transmission interleaving
static volatile enum eCBLidar lidar_primary_rx = eLidarFront;

// USBD Configuration Callback
static usbd_respond udev_setconf(usbd_device *dev, uint8_t cfg);

// External Interfacing
static struct usbi usbi;
static StaticQueue_t usbi_lidar_rx_front_sqh;
static struct udev_pkt_lidar usbi_lidar_rx_front_buf[USBI_LIDAR_BUF_SIZE] = {0};
static StaticQueue_t usbi_lidar_rx_vertical_sqh = {0};
static struct udev_pkt_lidar usbi_lidar_rx_vertical_buf[USBI_LIDAR_BUF_SIZE] = {
    0};
static StaticQueue_t usbi_lidar_tx_front_sqh = {0};
static struct udev_pkt_lidar usbi_lidar_tx_front_buf[USBI_LIDAR_BUF_SIZE] = {0};
static StaticQueue_t usbi_lidar_tx_vertical_sqh = {0};
static struct udev_pkt_lidar usbi_lidar_tx_vertical_buf[USBI_LIDAR_BUF_SIZE] = {
    0};
static StaticQueue_t usbi_ctrl_tx_sqh = {0};
static struct udev_pkt_ctrl_tx usbi_ctrl_tx_buf[USBI_CTRL_BUF_SIZE] = {0};
static StaticQueue_t usbi_ctrl_rx_sqh = {0};
static struct udev_pkt_ctrl_rx usbi_ctrl_rx_buf[USBI_CTRL_BUF_SIZE] = {0};

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

    // Enable USB OTG Interrupt
    NVIC_SetPriority(OTG_FS_IRQn, NVIC_Priority_MIN);
    NVIC_EnableIRQ(OTG_FS_IRQn);
    // Enable the USB Device
    usbd_enable(&udev, 1);
    usbd_connect(&udev, 1);

    // External Interface Setup

    usbi.lidar_rx_front =
        xQueueCreateStatic(USBI_LIDAR_BUF_SIZE,
                           sizeof(struct udev_pkt_lidar),
                           (uint8_t *) usbi_lidar_rx_front_buf,
                           &usbi_lidar_rx_front_sqh);

    usbi.lidar_rx_vertical =
        xQueueCreateStatic(USBI_LIDAR_BUF_SIZE,
                           sizeof(struct udev_pkt_lidar),
                           (uint8_t *) usbi_lidar_rx_vertical_buf,
                           &usbi_lidar_rx_vertical_sqh);

    usbi.lidar_tx_front =
        xQueueCreateStatic(USBI_LIDAR_BUF_SIZE,
                           sizeof(struct udev_pkt_lidar),
                           (uint8_t *) usbi_lidar_tx_front_buf,
                           &usbi_lidar_tx_front_sqh);

    usbi.lidar_rx_vertical =
        xQueueCreateStatic(USBI_LIDAR_BUF_SIZE,
                           sizeof(struct udev_pkt_lidar),
                           (uint8_t *) usbi_lidar_tx_vertical_buf,
                           &usbi_lidar_tx_vertical_sqh);

    usbi.ctrl_tx = xQueueCreateStatic(USBI_CTRL_BUF_SIZE,
                                      sizeof(struct udev_pkt_ctrl_tx),
                                      (uint8_t *) usbi_ctrl_tx_buf,
                                      &usbi_ctrl_tx_sqh);

    usbi.ctrl_rx = xQueueCreateStatic(USBI_CTRL_BUF_SIZE,
                                      sizeof(struct udev_pkt_ctrl_rx),
                                      (uint8_t *) usbi_ctrl_rx_buf,
                                      &usbi_ctrl_rx_sqh);

    return &usbi;
}

// USBD RX/TX Callbacks: Control COM Port
// Triggered during Virtual Communications Interface events
static void ctrl_rxtx(usbd_device *dev, uint8_t evt, uint8_t ep) {
    BaseType_t higher_woken = pdFALSE;
    if (evt == usbd_evt_eprx) {
        struct udev_pkt_ctrl_tx pkt_ctrl_tx = {0};
        usbd_ep_read(
            dev, ep, (void *) &pkt_ctrl_tx, sizeof(struct udev_pkt_ctrl_tx));
        xQueueOverwriteFromISR(usbi.ctrl_tx, &pkt_ctrl_tx, &higher_woken);
    } else {
        struct udev_pkt_ctrl_rx pkt_ctrl_rx = {0};
        // char ar[] = "Hello1\nHello2\nHello3\nHello4\n";
        if (xQueueReceiveFromISR(usbi.ctrl_rx, &pkt_ctrl_rx, &higher_woken) ==
            pdTRUE) {
            usbd_ep_write(
                dev, ep, (void *) &pkt_ctrl_rx, sizeof(struct udev_pkt_ctrl_rx));
        } else {
            usbd_ep_write(dev, ep, (void *) &pkt_ctrl_rx, 0);
        }
    }
    portYIELD_FROM_ISR(higher_woken);
}
static void lidar_rxtx(usbd_device *dev, uint8_t evt, uint8_t ep) {
    BaseType_t higher_woken = pdFALSE;
    struct udev_pkt_lidar pkt_lidar = {0};
    if (evt == usbd_evt_eprx) {
        usbd_ep_read(
            dev, ep, (void *) &pkt_lidar, sizeof(struct udev_pkt_ctrl_tx));
        QueueHandle_t *q = &usbi.lidar_rx_vertical;
        if (pkt_lidar.hdr.id == eLidarFront) {
            q = &usbi.lidar_rx_front;
        }
        if (xQueueSendFromISR(*q, &pkt_lidar, &higher_woken) == errQUEUE_FULL) {
            xQueueOverwriteFromISR(*q, &pkt_lidar, &higher_woken);
        }
    } else {
        // Interleave lidar transmission over a single channel
        QueueHandle_t *q = &usbi.lidar_tx_vertical;
        if (lidar_primary_rx == eLidarFront) {
            *q = usbi.lidar_tx_front;
            lidar_primary_rx = eLidarVertical;
        } else {
            lidar_primary_rx = eLidarFront;
        }
        if (xQueueReceiveFromISR(*q, &pkt_lidar, &higher_woken) == pdTRUE) {
            usbd_ep_write(
                dev, ep, (void *) &pkt_lidar, sizeof(struct udev_pkt_ctrl_rx));
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

        // TODO: Add back these functions
        usbd_reg_endpoint(dev, CTRL_RXD_EP, ctrl_rxtx);
        usbd_reg_endpoint(dev, CTRL_TXD_EP, ctrl_rxtx);

        usbd_ep_write(dev, CTRL_TXD_EP, 0, 0);
        usbd_ep_write(dev, CTRL_TXD_EP, 0, 0);
        return usbd_ack;
    default:
        return usbd_fail;
    }
}

// Interrupt handler for USB OTG FS
void OTG_FS_IRQHandler(void) {
    usbd_poll(&udev);
}
