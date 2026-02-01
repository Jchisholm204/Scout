/**
 * @file drone_defs.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief Common definitions for the drone
 * @version 0.1
 * @date Created: 2025-10-17
 * @modified Last Modified: 2025-10-17
 *
 * @copyright Copyright (c) 2025
 */

#ifndef _DRONE_DEFS_H_
#define _DRONE_DEFS_H_
#include "FreeRTOS.h"
#include "os/math.h"
#include "queue.h"

#include <stdio.h>

#define CTRL_BUF_SIZE 1

// Control Vector Structure
typedef union {
    struct {
        double x, y, z, w;
    };
    double data[4];
} ctrl_vec_t;

typedef struct {
    ctrl_vec_t cv;
    float ground_distance;
    float ceil_distance;
} ctrl_state_t;

typedef QueueHandle_t CtrlQueueHndl_t;

struct ctrl_queue {
    CtrlQueueHndl_t hndl;
    StaticQueue_t static_queue;
    ctrl_state_t storage_area[CTRL_BUF_SIZE];
};

static inline CtrlQueueHndl_t xCtrlQueueCreateStatic(struct ctrl_queue *cq) {
    if (!cq) {
        return NULL;
    }
    cq->hndl = xQueueCreateStatic(CTRL_BUF_SIZE,
                                  sizeof(ctrl_state_t),
                                  (uint8_t *) cq->storage_area,
                                  &cq->static_queue);
    return cq->hndl;
}

static inline ctrl_vec_t ctrl_vec_combine(ctrl_vec_t A,
                                          ctrl_vec_t B,
                                          float weight) {
    if (weight < -1)
        weight = -1;
    if (weight > 1)
        weight = 1;
    ctrl_vec_t C;
    for (int i = 0; i < 4; i++) {
        C.data[i] = (A.data[i] * (double) weight) +
                    (B.data[i] * (1.0 - (double) weight));
    }
    return C;
}

static inline ctrl_vec_t ctrl_vec_normal(ctrl_vec_t v) {
    double sum = 0;
    for (int i = 0; i < 4; i++)
        sum += v.data[i] * v.data[i];
    sum = (double)sqrtf((float)sum);
    for (int i = 0; i < 4; i++)
        v.data[i] = v.data[i] / sum;
    return v;
}

static inline void ctrl_vec_print(const char *prefix, ctrl_vec_t v) {
    printf("%s: %.3f %.3f %.3f %.3f\n", prefix, v.x, v.y, v.z, v.w);
}

#endif
