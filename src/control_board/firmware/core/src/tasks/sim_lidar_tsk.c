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
        for (int j = 0; j < 4; j++) {
            pHndl->sums_front[i].data[j] = 0.0f;
            pHndl->sums_vertical[i].data[j] = 0.0f;
        }
    }

    return pHndl->cvtx.hndl;
}

static inline float arm_sqrtf(float val) {
    float res;
    __asm volatile("vsqrt.f32 %0, %1" : "=t"(res) : "t"(val));
    return res;
}

void vSimLidarTsk(void *pvParams) {
    struct sim_lidar_tsk *pHndl = (struct sim_lidar_tsk *) pvParams;

    printf("Gplan Online\n");

    TickType_t last_wake_time = xTaskGetTickCount();

    int count = 0;

    float d_ground = 0;
    float d_ceil = 0;

    for (;;) {
        // Attempt to pull the latest packet from the incoming process queue
        static struct udev_pkt_lidar ldrpkt = {0};
        if (xQueueReceive(pHndl->usb.rx, &ldrpkt, 100) != pdTRUE) {
            // Input Queue Empty
            printf("Lidar Input Queue Empty\n");
            continue;
        }

        // Check that the packet has a valid seq number
        if (ldrpkt.hdr.sequence >= UDEV_LIDAR_SEQ_MAX) {
            continue;
        }

        // // Process the incoming data
        float sum_x = 0.0f;
        float sum_y = 0.0f;
        int valid_points = 0;
        for (int i = 0; i < ldrpkt.hdr.len; i++) {
            float d = (float) ldrpkt.distances[i] / 4000.0f;
            if (d > 45.0f || d < 0.1f) {
                continue;
            }
            valid_points++;
            float angle = udev_lidar_angle(ldrpkt.hdr.sequence, i);
            float weight = 1.0f / d;
            sum_x += weight * cosf(angle);
            sum_y += weight * sinf(angle);
        }

        // Dont bother processing packets with no points
        if (valid_points == 0) {
            continue;
        }

        // Add the new resultant sum depending on lidar orientation
        // if (ldrpkt.hdr.id == eLidarFront) {
        //     pHndl->sums_front[ldrpkt.hdr.sequence].x = sum_x;
        //     pHndl->sums_front[ldrpkt.hdr.sequence].y = sum_y;
        //     pHndl->sums_front[ldrpkt.hdr.sequence].z = 0.0f;
        //     pHndl->sums_front[ldrpkt.hdr.sequence].w = 0.0f;
        // }
        if (ldrpkt.hdr.id == eLidarVertical) {
            pHndl->sums_vertical[ldrpkt.hdr.sequence].x = 0.0f;
            pHndl->sums_vertical[ldrpkt.hdr.sequence].y = sum_y;
            pHndl->sums_vertical[ldrpkt.hdr.sequence].z = sum_x;
            pHndl->sums_vertical[ldrpkt.hdr.sequence].w = 0.0f;
        } else {
            continue;
        }

        // if (ldrpkt.hdr.id == eLidarVertical && ldrpkt.hdr.sequence == 3) {
        //     d_ground = (float) ldrpkt.distances[0] / 4000.0f;
        //     // printf("Ground Distance: %3.3f\n",
        //     //        ((float) ldrpkt.distances[0] / 4000.0f));
        // }
        //
        // if (ldrpkt.hdr.id == eLidarVertical && ldrpkt.hdr.sequence == 0) {
        //     d_ceil = (float) ldrpkt.distances[0] / 4000.0f;
        //     // printf("Ground Distance: %3.3f\n",
        //     //        ((float) ldrpkt.distances[0] / 4000.0f));
        // }

        // Calculate the collision vector
        ctrl_state_t cs;
        cs.cv.x = 0;
        cs.cv.y = 0;
        cs.cv.z = 0;
        cs.cv.w = 0;

        for (int i = 0; i < UDEV_LIDAR_SEQ_MAX; i++) {
            for (int j = 0; j < 4; j++) {
                cs.cv.data[j] += pHndl->sums_vertical[i].data[j];
                // qv.data[j] += pHndl->sums_front[i].data[j];
            }
        }

        // if (count >= 20) {
        // if (ldrpkt.hdr.id == eLidarVertical) {
        // count = 0;
        // if (ldrpkt.hdr.sequence == 3)
        // printf("S: %d X: %2.3f Y: %2.3f\n",
        //        ldrpkt.hdr.sequence,
        //        sum_x,
        //        sum_y);
        // printf("CV: ");
        // for (int j = 0; j < 4; j++) {
        //     printf("%2.4f ", qv.data[j]);
        // }
        // printf("\n");
        // }
        // count++;
        // Send CV to control task
        xQueueOverwrite(pHndl->cvtx.hndl, &cs);

        // float sum_sqrt = 0;
        // for (int j = 0; j < 4; j++) {
        //     sum_sqrt += qv.data[j] * qv.data[j];
        // }
        // sum_sqrt = arm_sqrtf(sum_sqrt);
        // for (int j = 0; j < 4; j++) {
        //     qv.data[j] = qv.data[j] / sum_sqrt;
        // }

        // Send the newly processed packet over USB for ROS LaserScan
        (void) xQueueSendToBack(pHndl->usb.tx, &ldrpkt, 10);
        // vTaskDelayUntil(&last_wake_time, 20);
        // quat_t qt;
        // qt.z = (d_ceil - d_ground) / ((d_ceil + d_ground));

        // printf("D: %2.3f\n", qt.z);
    }
}
