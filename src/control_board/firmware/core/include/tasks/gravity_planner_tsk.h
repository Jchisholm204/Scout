/**
 * @file gravity_planner_tsk.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2025-10-17
 * @modified Last Modified: 2025-10-17
 *
 * @copyright Copyright (c) 2025
 */

// LEGACY - Left alone for hendrix
#ifndef _GRAVITY_PLANNER_H_
#define _GRAVITY_PLANNER_H_
#warning "DEPRICATED"
#include "FreeRTOS.h"
#include "config/sys_cfg.h"
#include "drone_defs.h"
#include "queue.h"
#include "semphr.h"
#include "usb_lidar.h"
#include "usb_packet.h"

#include <stdio.h>

// Must be 1 to only hold most recent value
#define GPLAN_CVTX_BUF_SIZE 1

#define GPLAN_TSK_STACK_SIZE (configMINIMAL_STACK_SIZE << 2)

struct gplan_tsk {

    // Task information
    struct {
        TaskHandle_t hndl;
        StaticTask_t static_tsk;
        StackType_t stack[GPLAN_TSK_STACK_SIZE];
    } tsk;

    // Output Queue
    struct ctrl_queue cv_tx;

    struct {
        QueueHandle_t rx, tx;
    } usb;

    ctrl_vec_t sums_front[UDEV_LIDAR_SEQ_MAX];
    ctrl_vec_t sums_vertical[UDEV_LIDAR_SEQ_MAX];
};

extern QueueHandle_t gplan_tsk_init(struct gplan_tsk *pHndl,
                                    QueueHandle_t usb_rx,
                                    QueueHandle_t usb_tx);

#endif
