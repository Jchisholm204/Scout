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

    for (;;) {
        // Attempt to pull the latest packet from the incoming process queue
        static struct udev_pkt_lidar ldrpkt = {0};
        if (xQueueReceive(pHndl->usb.rx, &ldrpkt, 500) != pdTRUE) {
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
        float ground_sum = 0.0f;
        float ceil_sum = 0.0f;
        int valid_points = 0;
        for (int i = 0; i < ldrpkt.hdr.len; i++) {
            float d = (float) ldrpkt.distances[i] / 4000.0f;
            if (d > 45.0f || d < 0.1f) {
                continue;
            }
            valid_points++;
            float angle = udev_lidar_angle(ldrpkt.hdr.sequence, i);
            float weight = 1.0f / d;
            float cos = cosf(angle);
            sum_x += weight * cos;
            if (cos < 0) {
                ground_sum += fabsf(d * cos);
            } else {
                ceil_sum += fabsf(d * cos);
            }
            sum_y += weight * sinf(angle);
        }

        // Dont bother processing packets with no points
        if (valid_points == 0) {
            continue;
        }

        // ground_sum /= ((float) valid_points / 2.0f);
        // ceil_sum /= ((float) valid_points / 2.0f);

        // Add the new resultant sum depending on lidar orientation
        if (ldrpkt.hdr.id == eLidarFront) {
            pHndl->sums_front[ldrpkt.hdr.sequence].x = sum_x;
            pHndl->sums_front[ldrpkt.hdr.sequence].y = sum_y;
            pHndl->sums_front[ldrpkt.hdr.sequence].z = 0.0f;
            pHndl->sums_front[ldrpkt.hdr.sequence].w = 0.0f;
        }
        if (ldrpkt.hdr.id == eLidarVertical) {
            pHndl->sums_vertical[ldrpkt.hdr.sequence].x = 0.0f;
            pHndl->sums_vertical[ldrpkt.hdr.sequence].y = sum_y;
            pHndl->sums_vertical[ldrpkt.hdr.sequence].z = sum_x;
            pHndl->sums_vertical[ldrpkt.hdr.sequence].w = 0.0f;
            pHndl->ceil_sums[ldrpkt.hdr.sequence] = ceil_sum;
            pHndl->ground_sums[ldrpkt.hdr.sequence] = ground_sum;
        }

        // Calculate the collision vector
        ctrl_state_t cs;
        cs.cv.x = 0;
        cs.cv.y = 0;
        cs.cv.z = 0;
        cs.cv.w = 0;
        cs.ground_distance = 0;
        cs.ceil_distance = 0;

        for (int i = 0; i < UDEV_LIDAR_SEQ_MAX; i++) {
            for (int j = 0; j < 4; j++) {
                cs.cv.data[j] += pHndl->sums_vertical[i].data[j];
                cs.cv.data[j] += pHndl->sums_front[i].data[j];
            }
            cs.ground_distance += pHndl->ground_sums[i];
            cs.ceil_distance += pHndl->ceil_sums[i];
        }

        cs.cv = ctrl_vec_normal(cs.cv);

        // Send CV to control task
        xQueueOverwrite(pHndl->cvtx.hndl, &cs);

        // Send the newly processed packet over USB for ROS LaserScan
        (void) xQueueSendToBack(pHndl->usb.tx, &ldrpkt, 10);
    }
}
