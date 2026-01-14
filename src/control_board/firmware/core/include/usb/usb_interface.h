/**
 * @file usb_interface.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2025-10-17
 * @modified Last Modified: 2025-10-17
 *
 * @copyright Copyright (c) 2025
 */

#ifndef _USB_INTERFACE_H_
#define _USB_INTERFACE_H_

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "usb_cb_defs.h"
#include "usb_packet.h"

// Lidar Buffer size (number of usb packets)
#define USBI_LIDAR_BUF_SIZE 8

// Control Buffer size
// Only keep the most recent control packet
#define USBI_CTRL_BUF_SIZE 1

struct usbi {
    // RX/TX From side of Device
    QueueHandle_t lidar_rx;
    QueueHandle_t lidar_tx;
    // RX/TX From side of host
    QueueHandle_t ctrl_tx;
    QueueHandle_t ctrl_rx;
};

extern struct usbi * usbi_init(void);

#endif
