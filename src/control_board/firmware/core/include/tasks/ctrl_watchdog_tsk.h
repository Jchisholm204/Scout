/**
 * @file ctrl_watchdog_tsk.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief 
 * @version 0.1
 * @date Created: 2026-01-31
 * @modified Last Modified: 2026-01-31
 *
 * @copyright Copyright (c) 2026
 */

#ifndef _CTRL_WATCHDOG_TSK_H_
#define _CTRL_WATCHDOG_TSK_H_
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "ctrl_tsk.h"

// Control Task Watchdog Entity
struct ctrl_watchdog {
    // Task information
    struct {
        TaskHandle_t hndl;
        StaticTask_t static_task;
        StackType_t stack[configMINIMAL_STACK_SIZE];
    } tsk;

    struct ctrl_tsk *pCtrlTsk;
};

#endif
