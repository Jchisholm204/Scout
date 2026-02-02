/**
 * @file sim_lidar_tsk.c
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.2
 * @date Created: 2025-10-17
 * @modified Last Modified: 2026-01-16
 *
 * @copyright Copyright (c) 2025
 */

#include "tasks/sim_lidar_tsk.h"

#include "FreeRTOS.h"

#include <math.h>

void vSimLidarTsk(void *pvParams);

CtrlQueueHndl_t sim_lidar_tsk_init(struct sim_lidar_tsk *pHndl,
                                   QueueHandle_t usb_rx,
                                   QueueHandle_t usb_tx) {
    if (!pHndl) {
        return NULL;
    }
    // Initialize the usb queues
    pHndl->usb.rx = usb_rx;
    pHndl->usb.tx = usb_tx;

    // Setup the Collision Vector Output Queue
    if (!xCtrlQueueCreateStatic(&pHndl->cvtx)) {
        return NULL;
    }

    // Setup the Gravity Planner Task
    pHndl->tsk.hndl = xTaskCreateStatic(vSimLidarTsk,
                                        "gplan",
                                        SIM_LIDAR_TSK_STACK_SIZE,
                                        pHndl,
                                        configMAX_PRIORITIES - 4,
                                        pHndl->tsk.stack,
                                        &pHndl->tsk.static_tsk);

    if (!pHndl->tsk.hndl) {
        return NULL;
    }

    // Setup the lidar data storage containers
    for (int i = 0; i < UDEV_LIDAR_SEQ_MAX; i++) {
        pHndl->dist_avgs_vert[i] = 0;
        pHndl->dist_avgs_front[i] = 0;
    }

    return pHndl->cvtx.hndl;
}

void vSimLidarTsk(void *pvParams) {
    struct sim_lidar_tsk *pHndl = (struct sim_lidar_tsk *) pvParams;

    printf("Gplan Online\n");

    for (;;) {
        // Attempt to pull the latest packet from the incoming process queue
        static struct udev_pkt_lidar ldrpkt = {0};
        if (xQueueReceive(pHndl->usb.rx, &ldrpkt, 500) != pdTRUE) {
            // Input Queue Empty
            printf("Lidar Input Queue Empty\n");
            continue;
        }

        int seqn = ldrpkt.hdr.sequence;

        // Check that the packet has a valid seq number
        if (seqn >= UDEV_LIDAR_SEQ_MAX) {
            continue;
        }

        // Process the incoming data
        int valid_points = 0;
        double dist_avg = 0;
        for (int i = 0; i < ldrpkt.hdr.len; i++) {
            float d = (float) ldrpkt.distances[i] / 4000.0f;
            if (d > 12.0f || d < 0.5f) {
                continue;
            }
            valid_points++;
            dist_avg += (double) d;
        }

        // Dont bother processing packets with no points
        if (valid_points == 0) {
            continue;
        }

        dist_avg /= (double) valid_points;

        // Add the new resultant sum depending on lidar orientation
        if (ldrpkt.hdr.id == eLidarFront) {
            pHndl->dist_avgs_front[seqn] = dist_avg;
        }
        if (ldrpkt.hdr.id == eLidarVertical) {
            pHndl->dist_avgs_vert[seqn] = dist_avg;
        }

        double v_radius = 0;
        double h_radius = 0;
        for (int i = 0; i < UDEV_LIDAR_SEQ_MAX; i++) {
            double d_v = pHndl->dist_avgs_vert[i];
            double d_h = pHndl->dist_avgs_front[i];
            v_radius += d_v;
            h_radius += d_h;
        }

        v_radius /= ((double) UDEV_LIDAR_SEQ_MAX);
        h_radius /= ((double) UDEV_LIDAR_SEQ_MAX);

        ctrl_state_t cs = {0};
        for (int i = 0; i < UDEV_LIDAR_SEQ_MAX; i++) {
            double d_v = pHndl->dist_avgs_vert[i];
            if (d_v > v_radius) {
                d_v = 0;
            } else {
                d_v = (v_radius - d_v);
            }
            double d_h = pHndl->dist_avgs_front[i];
            if (d_h > h_radius) {
                d_h = 0;
            } else {
                d_h = (h_radius - d_h);
            }
            float angle = udev_lidar_angle(i, UDEV_LIDAR_POINTS / 2);
            cs.cv.x += ((double) cosf(angle) * (d_h));
            cs.cv.y += ((double) sinf(angle) * (d_h));
            cs.cv.y += ((double) sinf(angle) * (d_v));
            cs.cv.z += ((double) cosf(angle) * (d_v));
        }

        cs.cv.x /= ((double) UDEV_LIDAR_SEQ_MAX);
        cs.cv.y /= ((double) UDEV_LIDAR_SEQ_MAX);
        cs.cv.y /= ((double) UDEV_LIDAR_SEQ_MAX);
        cs.cv.z /= ((double) UDEV_LIDAR_SEQ_MAX);

        cs.ground_distance =
            (float) (pHndl->dist_avgs_vert[2] + pHndl->dist_avgs_vert[3]) /
            2.0f;
        cs.ceil_distance =
            (float) (pHndl->dist_avgs_vert[0] + pHndl->dist_avgs_vert[5]) /
            2.0f;

        cs.radius = (float) (v_radius + h_radius) / 2.0f;

        // Send CV to control task
        xQueueOverwrite(pHndl->cvtx.hndl, &cs);

        // Send the newly processed packet over USB for ROS LaserScan
        (void) xQueueSendToBack(pHndl->usb.tx, &ldrpkt, 10);
    }
}
