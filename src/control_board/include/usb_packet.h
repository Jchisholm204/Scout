/**
 * @file usb_packet.h
 * @author Jacob Chisholm (Jchisholm204.github.io)
 * @brief USB Packet Declarations
 * @version 0.1
 * @date 2026-01-11
 * @modified Last Modified: 2026-01-11
 *
 * @copyright Copyright (c) 2023
 *
 *
 * This file is used by the driver and the embedded device
 * Both must be recompiled/flashed when changed
 *
 * RX/TX notation is from the host perspective, as specified by USB-IF
 *
 */

#ifndef _USB_PACKET_H_
#define _USB_PACKET_H_

#include "usb_dev.h"

#include <assert.h>
#include <stdint.h>

struct udev_pkt_ctrl_tx {
    union {
        float data[3];
        struct {
            float x, y, z;
        };
    } vel;
    uint8_t mode;
} __attribute__((packed));

struct udev_pkt_ctrl_rx {
    union {
        float data[3];
        struct {
            float x, y, z;
        };
    } vel;
    uint8_t vBatt;
    uint8_t rssi;
    uint8_t status;
    uint8_t mode;
} __attribute__((packed));

struct udev_pkt_lidar {
    struct {
        uint8_t id : 1;
        uint8_t sequence : 7;
    } __attribute__((packed)) hdr;
    float data[15];
} __attribute__((packed));

// USB Packets must be less than 0x40/64 bytes in length
static_assert(sizeof(struct udev_pkt_ctrl_tx) <= CTRL_DATA_SZ,
              "USBD Control Packet Oversize");
static_assert(sizeof(struct udev_pkt_ctrl_rx) <= CTRL_DATA_SZ,
              "USBD Control Packet Oversize");
static_assert(sizeof(struct udev_pkt_lidar) <= CTRL_DATA_SZ,
              "USBD Status Packet Oversize");

#endif
