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
#include "queue.h"

#define CTRL_BUF_SIZE 1

// Control Vector Structure
typedef union {
    struct {
        float x, y, z, w;
    };
    float data[4];
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

#endif
