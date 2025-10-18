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

#include "usb_interface.h"

#include "FreeRTOS.h"
#include "config/FreeRTOSConfig.h"
#include "config/nvic.h"
#include "config/pin_cfg.h"
#include "drivers/stusb/usb.h"
#include "hal/hal_usb.h"
#include "stm32f446xx.h"
#include "task.h"
#include "usb_desc.h"
#include "usb_packet.h"

#define USB_STACK_SIZE (configMINIMAL_STACK_SIZE << 2)

// USB Device
usbd_device udev;
uint32_t usb0_buf[CDC_EP0_SIZE]; // EP0 Buffer
// USB Info/Ctrl Packet
static volatile struct udev_pkt_status udev_info;
static volatile struct udev_pkt_ctrl udev_ctrl;
// USB VCOM Buffers
static volatile uint8_t vcom_rxBuf[VCOM_DATA_SZ] = {0};
static volatile uint8_t vcom_txBuf[VCOM_DATA_SZ] = {0};
static volatile uint16_t vcom_txSize = 0;
// USBD Configuration Callback
static usbd_respond udev_setconf(usbd_device* dev, uint8_t cfg);

// USB Task Handle
// static TaskHandle_t usbHndl;
// static StackType_t puUsbStack[USB_STACK_SIZE];
// static StaticTask_t pxUsbTsk;


void usbi_init(void) {
    // Initialize the USB Device
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
}

// USBD RX/TX Callbacks: Virtual COM Port
// Triggered during Virtual Communications Interface events
static void vcom_rxtx(usbd_device* dev, uint8_t evt, uint8_t ep) {
    if (evt == usbd_evt_eprx) {
        usbd_ep_read(dev, ep, (void*) vcom_rxBuf, VCOM_DATA_SZ);
    } else {
        usbd_ep_write(dev, ep, (void*) &vcom_txBuf, vcom_txSize);
        vcom_txSize = 0;
    }
}

static usbd_respond udev_setconf(usbd_device* dev, uint8_t cfg) {
    switch (cfg) {
    case 0:
        /* deconfiguring device */
        usbd_ep_deconfig(dev, VCOM_NTF_EP);
        usbd_ep_deconfig(dev, VCOM_TXD_EP);
        usbd_ep_deconfig(dev, VCOM_RXD_EP);
        usbd_ep_deconfig(dev, CTRL_NTF_EP);
        usbd_ep_deconfig(dev, CTRL_TXD_EP);
        usbd_ep_deconfig(dev, CTRL_RXD_EP);
        usbd_reg_endpoint(dev, VCOM_RXD_EP, 0);
        usbd_reg_endpoint(dev, VCOM_TXD_EP, 0);
        usbd_reg_endpoint(dev, CTRL_RXD_EP, 0);
        usbd_reg_endpoint(dev, CTRL_TXD_EP, 0);
        return usbd_ack;
    case 1:
        /* configuring device */
        usbd_ep_config(dev,
                       VCOM_RXD_EP,
                       USB_EPTYPE_BULK /*| USB_EPTYPE_DBLBUF*/,
                       VCOM_DATA_SZ);
        usbd_ep_config(dev,
                       VCOM_TXD_EP,
                       USB_EPTYPE_BULK /*| USB_EPTYPE_DBLBUF*/,
                       VCOM_DATA_SZ);
        usbd_ep_config(dev, VCOM_NTF_EP, USB_EPTYPE_INTERRUPT, VCOM_NTF_SZ);
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
        usbd_reg_endpoint(dev, VCOM_RXD_EP, vcom_rxtx);
        usbd_reg_endpoint(dev, VCOM_TXD_EP, vcom_rxtx);
        // usbd_reg_endpoint(dev, CTRL_RXD_EP, ctrl_rxtx);
        // usbd_reg_endpoint(dev, CTRL_TXD_EP, ctrl_rxtx);

        usbd_ep_write(dev, VCOM_TXD_EP, 0, 0);
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
