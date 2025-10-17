/**
 * @file main.c
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief ELEC 498 Capstone - Central Control Board
 * @version 0.1
 * @date Created: 2025-1-19
 * @modified Last Modified: 2025-10-14
 *
 * @copyright Copyright (c) 2025
 */

#include "main.h"

#include "FreeRTOS.h"
#include "config/FreeRTOSConfig.h"
#include "config/pin_cfg.h"
#include "drivers/canbus.h"
#include "drivers/serial.h"
#include "hal/hal_usb.h"
#include "stm32f446xx.h"
#include "systime.h"
#include "task.h"

#include <memory.h>
#include <stdio.h>
#include <string.h>

// USB Device Includes
#include "drivers/stusb/usb.h"
#include "test_tsks.h"
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
static TaskHandle_t usbHndl;
static StackType_t puUsbStack[USB_STACK_SIZE];
static StaticTask_t pxUsbTsk;

// USB Task
void vTskUSB(void* pvParams);

// Initialize all system Interfaces
void Init(void) {
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

    // Initialize UART
    serial_init(&Serial5, /*baud*/ 9600, PIN_UART5_RX, PIN_UART5_TX);
    serial_init(&Serial3, /*baud*/ 9600, PIN_USART3_RX, PIN_USART3_TX);

    /**
     * Initialize System Tasks...
     * All tasks should be initialized as static
     * Tasks can be initialized dynamically, but may crash the system if they
     * overflow the system memory (128Kb for the STM32f446)
     */
    usbHndl = xTaskCreateStatic(vTskUSB,
                                "USB",
                                USB_STACK_SIZE,
                                NULL,
                                /*Priority*/ 1,
                                puUsbStack,
                                &pxUsbTsk);
}

void vTskUSB(void* pvParams) {
    (void) (pvParams);
    char msg[] = "USB Task Online";
    memcpy((void*) vcom_txBuf, msg, sizeof(msg));
    vcom_txSize = sizeof(msg);
    struct systime time;
    gpio_set_mode(PIN_LED1, GPIO_MODE_OUTPUT);
    gpio_set_mode(PIN_LED2, GPIO_MODE_OUTPUT);
    gpio_write(PIN_LED1, true);
    // hal_uart_init(UART5, 9600, PIN_UART5_RX, PIN_UART5_TX);
    for (;;) {
        systime_fromTicks(xTaskGetTickCount(), &time);
        systime_getStr(&time);
        // fprintf(Serial5.fp, "Time: %s\n", time.str);
        eSerialError e;
        if ((e = serial_write(&Serial5, msg, sizeof(msg), 10)) != eSerialOK)
            vcom_txSize = (uint16_t)snprintf((char*) vcom_txBuf,
                                   VCOM_DATA_SZ,
                                   "Serial 5 Write Fail (%d)\n",
                                   e);
        if (e != eSerialOK || (e = serial_write(&Serial3, msg, sizeof(msg), 10)) != eSerialOK)
            vcom_txSize = (uint16_t)snprintf((char*) vcom_txBuf,
                                   VCOM_DATA_SZ,
                                   "Serial 3 Write Fail (%d)\n",
                                   e);
        else {
            memcpy((void*) vcom_txBuf, time.str, SYSTIME_STR_LEN);
            vcom_txSize = SYSTIME_STR_LEN;
        }

        // hal_uart_write_buf(UART5, msg, sizeof(msg));

        gpio_toggle_pin(PIN_LED1);
        gpio_toggle_pin(PIN_LED2);
        vTaskDelay(500);
    }
}

// Triggered when the USB Host provides data to the control interface
static void ctrl_rx(usbd_device* dev, uint8_t evt, uint8_t ep) {
    (void) evt;
    usbd_ep_read(dev, ep, (void*) &udev_ctrl, sizeof(struct udev_pkt_ctrl));
    // Handle Control rx event
}

// Triggered when the USB Host requests data from the control interface
static void ctrl_tx(usbd_device* dev, uint8_t evt, uint8_t ep) {
    (void) evt;
    // Get the info

    // Write back to the USB Memory
    usbd_ep_write(dev, ep, (void*) &udev_info, sizeof(struct udev_pkt_status));
}

// USBD RX/TX Callbacks: Control
// Triggered during Control Interface events
static void ctrl_rxtx(usbd_device* dev, uint8_t evt, uint8_t ep) {
    if (evt == usbd_evt_eprx)
        ctrl_rx(dev, evt, ep);
    else
        ctrl_tx(dev, evt, ep);
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
        usbd_reg_endpoint(dev, CTRL_RXD_EP, ctrl_rxtx);
        usbd_reg_endpoint(dev, CTRL_TXD_EP, ctrl_rxtx);

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

int main(void) {
    // Call the init function
    Init();

    // Start Scheduler: Runs tasks initialized above
    vTaskStartScheduler();

    // System Main loop (Should never run -> Scheduler runs infinitely)
    for (;;) {
        asm("nop");
        // gpio_write(PIN_LED2, true);
    }
    return 0;
}
