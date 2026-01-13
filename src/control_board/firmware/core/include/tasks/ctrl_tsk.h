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
#include "drivers/serial.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "protocols/crsf/crsf.h"

#include <stdio.h>

struct ctrl_tsk {
    Serial_t *pSerial;
    CRSF_t crsf;

    // Task information
    TaskHandle_t tsk_hndl;
    StaticTask_t tsk_buf;
    StackType_t tsk_stack[configMINIMAL_STACK_SIZE];

    // Recieve Buffer (from serial driver interrupt)
    StreamBufferHandle_t tx_hndl;
    StaticStreamBuffer_t tx_streamBuf;
    uint8_t tx_buf[configMINIMAL_STACK_SIZE];
};

extern int ctrl_tsk_init(struct ctrl_tsk *pHndl, Serial_t *pSerial);

#endif
