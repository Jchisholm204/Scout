/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"

#include "hal/hal_uart.h"
#include "main.h"
#include "task.h"

#include <stdio.h>

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize);
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
// uint32_t ulGetRunTimeCounterValue(void);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize) {
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    // You can add logging or a debugging breakpoint here
    // Example: Print the task name that caused the overflow
    char msg[] = "Stack overflow in task:";
    for (int i = 0; i < sizeof(msg); i++) {
        ITM_SendChar(msg[i]);
    }
    for (int i = 0; i < 20; i++) {
        ITM_SendChar(pcTaskName[i]);
        if(pcTaskName[i] == '\0'){
            break;
        }
    }

    // Optionally, halt the system or enter an infinite loop
    while (1) {
        // Halt the system or take appropriate actions
    }
}
