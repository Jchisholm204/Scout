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

#ifndef _GRAVITY_PLANNER_H_
#define _GRAVITY_PLANNER_H_
#include "FreeRTOS.h"
#include "config/sys_cfg.h"
#include "drone_defs.h"
#include "usb_packet.h"
#include "queue.h"
#include "semphr.h"

#include <stdio.h>

// Must be 1 to only hold most recent value
#define GPLAN_CVTX_BUF_SIZE 1

struct gplan_tsk {

    // Task information
    struct {
        TaskHandle_t hndl;
        StaticTask_t static_tsk;
        StackType_t stack[configMINIMAL_STACK_SIZE];
    } tsk;

    // Output Queue
    struct {
        QueueHandle_t hndl;
        StaticQueue_t static_queue;
        quat_t buf[GPLAN_CVTX_BUF_SIZE];
    } cv_tx;

    struct {
        QueueHandle_t rx_front;
        QueueHandle_t rx_vertical;
        QueueHandle_t tx_front;
        QueueHandle_t tx_vertical;
    } usb;
};

extern QueueHandle_t gplan_tsk_init(struct gplan_tsk *pHndl,
                                    QueueHandle_t usb_rx_front,
                                    QueueHandle_t usb_rx_vertical,
                                    QueueHandle_t usb_tx_front,
                                    QueueHandle_t usb_tx_vertical);

#endif
