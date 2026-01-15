/**
 * @file tsk_testing.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date 2025-01-17
 *
 * @copyright Copyright (c) 2025
 *
 */

#ifndef _TEST_TSKS_H_
#define _TEST_TSKS_H_
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

struct test_tsk {
    struct {
        TaskHandle_t hndl;
        StaticTask_t static_tsk;
        StackType_t stack[configMINIMAL_STACK_SIZE];
    } tsk;
    
    TickType_t delay_ticks;
};

int test_tsk_init(struct test_tsk *pHndl, TickType_t delay_ticks);

#endif
