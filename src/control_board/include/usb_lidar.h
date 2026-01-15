/**
 * @file usb_lidar.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief USB Lidar Packet Transmission Helpers
 * @version 0.1
 * @date Created: 2026-01-14
 * @modified Last Modified: 2026-01-14
 *
 * @copyright Copyright (c) 2026
 */

#ifndef _USB_LIDAR_H_
#define _USB_LIDAR_H_

#include "string.h"
#include "usb_cb_defs.h"
#include "usb_packet.h"

#define UDEV_LIDAR_SEQ_STEP ((float) (3.1415926535f / (int) UDEV_LIDAR_SEQ_MAX))
#define UDEV_LIDAR_ITER_STEP ((float) (UDEV_LIDAR_SEQ_STEP / (int) UDEV_LIDAR_POINTS))

struct udev_lidar {
    enum eCBLidar lidar;
    int most_recent;
    struct udev_pkt_lidar data[UDEV_LIDAR_SEQ_MAX];
};

/**
 * @brief Calculate the angle of a given seq number and iterator
 *
 * @param seq Packet Sequence Number
 * @param iterator Packet Iterator Number
 * @return
 */
static inline float udev_lidar_angle(int seq, int iterator) {
    return (float) ((float) seq * UDEV_LIDAR_SEQ_STEP +
                    (float) iterator * UDEV_LIDAR_ITER_STEP);
}

/**
 * @brief Calculate the seq and iterator for a given angle
 * * @param angle    The input angle in radians
 * @param out_seq  Pointer to store the calculated sequence number
 * @param out_iter Pointer to store the calculated iterator/index
 */
static inline void udev_lidar_index(float angle, int* out_seq, int* out_iter) {
    int global_index = (int) ((angle / UDEV_LIDAR_ITER_STEP) + 0.5f);
    *out_seq = global_index / UDEV_LIDAR_POINTS;
    *out_iter = global_index % UDEV_LIDAR_POINTS;
}

/**
 * @brief Calculate the average influcence of the packet
 *
 * @param pkt
 * @return
 */
static inline float udev_lidar_sum(struct udev_pkt_lidar* pkt) {
    if (!pkt)
        return 0.0f;
    float sum = 0.0f;
    for (int i = 0; i < UDEV_LIDAR_POINTS; i++) {
        sum += ((float) pkt->distances[i]) / 4000.0f;
    }
    return sum / ((float) (int) UDEV_LIDAR_POINTS);
}

static inline void udev_lidar_init(struct udev_lidar* pHndl, enum eCBLidar lidar) {
    if (!pHndl) {
        return;
    }
    pHndl->lidar = lidar;
    pHndl->most_recent = -1;
    for (int i = 0; i < UDEV_LIDAR_SEQ_MAX; i++) {
        pHndl->data[i].hdr.sequence = i;
        pHndl->data[i].hdr.id = lidar;
        pHndl->data[i].hdr.len = 0;
        pHndl->data[i].distance_sum = 0;
        for (int j = 0; j < UDEV_LIDAR_POINTS; j++) {
            pHndl->data[i].distances[i] = 0;
        }
    }
}

/**
 * @brief Add a new packet into the current collection
 *
 * @param pHndl
 * @param pkt
 * @return
 */
static inline int udev_lidar_recv(struct udev_lidar* pHndl, struct udev_pkt_lidar* pkt) {
    if (!pHndl || !pkt) {
        return -1;
    }
    if (pkt->hdr.id != pHndl->lidar) {
        return -2;
    }
    if (pkt->hdr.sequence >= UDEV_LIDAR_SEQ_MAX) {
        return -3;
    }
    pHndl->most_recent = pkt->hdr.sequence;
    memcpy(&pHndl->data[pkt->hdr.sequence], pkt, sizeof(udev_pkt_lidar));
    return 0;
}

#endif
