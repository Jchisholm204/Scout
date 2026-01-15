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

#define UDEV_LIDAR_RANGE 180
#define UDEV_LIDAR_POINTS ((LIDAR_DATA_SZ - 2) / 2)
#define UDEV_SEQ_MAX ((UDEV_LIDAR_RANGE - 1 + UDEV_LIDAR_POINTS) / UDEV_LIDAR_POINTS)

struct udev_pkt_ctrl_tx {
    union {
        struct {
            float x, y, z, w;
        };
        float data[4];
    } vel;
    uint8_t mode;
} __attribute__((packed, aligned(4)));

struct udev_pkt_ctrl_rx {
    union {
        struct {
            float x, y, z, w;
        };
        float data[4];
    } vel;
    uint8_t vBatt;
    uint8_t rssi;
    uint8_t status;
    uint8_t mode;
} __attribute__((packed, aligned(4)));

struct udev_pkt_lidar {
    struct {
        uint8_t id : 1;
        uint8_t sequence : 7;
        uint8_t len;
    } __attribute__((packed)) hdr;
    // distance (mm) = distance / 4.0
    uint16_t distances[UDEV_LIDAR_POINTS];
} __attribute__((aligned(4)));

// USB Packets must be less than 0x40/64 bytes in length
static_assert(sizeof(struct udev_pkt_ctrl_tx) <= CTRL_DATA_SZ,
              "USBD Control Packet Oversize");
static_assert(sizeof(struct udev_pkt_ctrl_rx) <= CTRL_DATA_SZ,
              "USBD Control Packet Oversize");
static_assert(sizeof(struct udev_pkt_lidar) <= LIDAR_DATA_SZ,
              "USBD Status Packet Oversize");

#endif
