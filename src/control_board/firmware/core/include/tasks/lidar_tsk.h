/**
 * @file lidar_tsk.h
 * @author Hendrix Gryspeerdt
 * @brief
 * @version 0.1
 * @date Created: 2026-03-07
 * @modified Last Modified: 2026-03-07
 *
 * @copyright Copyright (c) 2026
 */

#ifndef _LIDAR_TSK_H_
#define _LIDAR_TSK_H_
#include "FreeRTOS.h"
#include "config/sys_cfg.h"
#include "drone_defs.h"
#include "queue.h"
#include "semphr.h"
#include "usb_lidar.h"
#include "usb_packet.h"
#include "protocols/rplidar/rplidar.h"

#include <stdio.h>

#define LIDAR_TSK_STACK_SIZE (configMINIMAL_STACK_SIZE << 2)

struct lidar_tsk {

    // Task information
    struct {
        TaskHandle_t hndl;
        StaticTask_t static_tsk;
        StackType_t stack[LIDAR_TSK_STACK_SIZE];
    } tsk;

    // Output Queue
    struct ctrl_queue cvtx;

    struct {
        QueueHandle_t rx, tx;
    } usb;

    // RPLiDAR Sensors
    RpLidar_t rplidar[2];


    ctrl_vec_t sums_front[UDEV_LIDAR_SEQ_MAX];
    ctrl_vec_t sums_vertical[UDEV_LIDAR_SEQ_MAX];
    float ground_sums[UDEV_LIDAR_SEQ_MAX];
    float ceil_sums[UDEV_LIDAR_SEQ_MAX];
};

extern CtrlQueueHndl_t lidar_tsk_init(struct lidar_tsk *pHndl,
                                        //   QueueHandle_t usb_rx,
                                          QueueHandle_t usb_tx);

#endif
