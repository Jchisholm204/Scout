/**
 * @file usb_dev.h
 * @author Jacob Chisholm (jchisholm204.github.io)
 * @brief Capstone Drone Control USB Device Definitions
 * @version 0.1
 * @date 2026-01-11
 * 
 * @copyright Copyright (c) 2023
 * 
 * USB Device Definitions:
 *  - Endpoint Definitions
 *  - USB Device IDs
 *
 * This file is used by the driver and the embedded device
 * Both must be recompiled/flashed when changed
 *
 */

#ifndef _USB_DEV_H_
#define _USB_DEV_H_

#ifndef UDEV_VERSION
#define UDEV_VERSION "1.0.0"
#endif

#define UDEV_INTERFACES  0x04

// Drone Control Endpoint
#define CTRL_RXD_EP      0x01
#define CTRL_TXD_EP      0x81
#define CTRL_DATA_SZ     0x40
#define CTRL_NTF_EP      0x82
#define CTRL_NTF_SZ      0x08
#define CTRL_NTF_INUM    0x00
#define CTRL_DATA_INUM   0x01

#define LIDAR_RXD_EP      0x03
#define LIDAR_TXD_EP      0x83
#define LIDAR_DATA_SZ     0x40
#define LIDAR_NTF_EP      0x84
#define LIDAR_NTF_SZ      0x08
#define LIDAR_NTF_INUM    0x02
#define LIDAR_DATA_INUM   0x03
//
// #define SIM_RXD_EP      0x05
// #define SIM_TXD_EP      0x85
// #define SIM_DATA_SZ     0x40
// #define SIM_NTF_EP      0x86
// #define SIM_NTF_SZ      0x08
// #define SIM_NTF_INUM    0x04
// #define SIM_DATA_INUM   0x05

// USB Device Vendor ID:
//  Use 0xFFFF or 0xFFFE as designated by the USBIF,
//  These vendor ID's are reserved for test or hobby devices and
//  will not conflict with registered vendor drivers
#define VENDOR_ID 0xFFFE
// USB Device Product ID:
//  Used to seperate usb devices from the same vendor
//  Must be different for each type of device
#define DEVICE_ID 0xD40C

#endif  // INCLUDE_INCLUDE_USB_DEV_H_

