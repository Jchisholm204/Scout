/**
 * @file sim_lidar_tsk.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2026-01-16
 * @modified Last Modified: 2026-01-16
 *
 * @copyright Copyright (c) 2026
 */

#ifndef _SIM_LIDAR_H_
#define _SIM_LIDAR_H_
#include "FreeRTOS.h"
#include "config/sys_cfg.h"
#include "drone_defs.h"
#include "queue.h"
#include "semphr.h"
#include "usb_lidar.h"
#include "usb_packet.h"

#include <stdio.h>

#define SIM_LIDAR_TSK_STACK_SIZE (configMINIMAL_STACK_SIZE << 2)

struct sim_lidar_tsk {

    // Task information
    struct {
        TaskHandle_t hndl;
        StaticTask_t static_tsk;
        StackType_t stack[SIM_LIDAR_TSK_STACK_SIZE];
    } tsk;

    // Output Queue
    struct ctrl_queue cvtx;

    struct {
        QueueHandle_t rx, tx;
    } usb;

    ctrl_vec_t sums_front[UDEV_LIDAR_SEQ_MAX];
    ctrl_vec_t sums_vertical[UDEV_LIDAR_SEQ_MAX];
    float ground_sums[UDEV_LIDAR_SEQ_MAX];
    float ceil_sums[UDEV_LIDAR_SEQ_MAX];
};

extern CtrlQueueHndl_t sim_lidar_tsk_init(struct sim_lidar_tsk *pHndl,
                                          QueueHandle_t usb_rx,
                                          QueueHandle_t usb_tx);

#endif
