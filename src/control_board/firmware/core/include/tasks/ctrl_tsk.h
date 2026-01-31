/**
 * @file ctrl_tsk.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2026-01-12
 * @modified Last Modified: 2026-01-12
 *
 * @copyright Copyright (c) 2026
 */

#ifndef _CTRL_TSK_H_
#define _CTRL_TSK_H_
#include "FreeRTOS.h"
#include "config/sys_cfg.h"
#include "controllers/antigravity.h"
#include "controllers/pid_controller.h"
#include "drivers/serial.h"
#include "protocols/crsf/crsf.h"
#include "queue.h"
#include "semphr.h"
#include "stream_buffer.h"

#include <stdio.h>

#define CTRL_TSK_STACK_SIZE (configMINIMAL_STACK_SIZE << 2)

struct ctrl_tsk {
    struct {
        CRSF_t crsf;
        // Recieve Buffer (from serial driver interrupt)
        StreamBufferHandle_t buf_hndl;
        StaticStreamBuffer_t stream_buffer;
        uint8_t storage_area[configMINIMAL_STACK_SIZE];
    } rc_crsf, fc_crsf;

    // Task information
    struct {
        TaskHandle_t hndl;
        StaticTask_t static_task;
        StackType_t stack[CTRL_TSK_STACK_SIZE];
    } tsk;

    struct {
        QueueHandle_t tx;
        QueueHandle_t rx;
    } usb;

    struct pid_controller pid_z;
    struct pid_controller pid_x;
    struct pid_controller pid_y;

    struct antigravity_controller antigrav;

    QueueHandle_t col_rx;
};

extern int ctrl_tsk_init(struct ctrl_tsk *pHndl,
                         Serial_t *rc_serial,
                         Serial_t *fc_serial,
                         QueueHandle_t usb_rx,
                         QueueHandle_t usb_tx,
                         QueueHandle_t col_rx);

#endif
