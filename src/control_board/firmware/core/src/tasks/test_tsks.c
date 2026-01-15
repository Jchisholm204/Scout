/**
 * @file test_tsks.c
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date 2024-10-20
 *
 * @copyright Copyright (c) 2024
 *
 * This file is used for testing - Disable this in production systems
 *
 * Anything here is not guaranteed to work, and most likely does not work
 */
#include "tasks/test_tsks.h"

#include "FreeRTOS.h"
#include "config/FreeRTOSConfig.h"
#include "config/pin_cfg.h"
#include "hal/hal_gpio.h"
#include "main.h"
#include "os/systime.h"
#include "stm32f446xx.h"
#include "task.h"

#include <stdio.h>

void vTsk_testOnline(void *pvParams);

int test_tsk_init(struct test_tsk *pHndl, TickType_t delay_ticks) {
    if (!pHndl) {
        return -1;
    }

    pHndl->tsk.hndl = xTaskCreateStatic(vTsk_testOnline,
                                        "test",
                                        configMINIMAL_STACK_SIZE,
                                        pHndl,
                                        0,
                                        pHndl->tsk.stack,
                                        &pHndl->tsk.static_tsk);
    if (!pHndl->tsk.hndl) {
        return -1;
    }

    pHndl->delay_ticks = delay_ticks;

    return 0;
}

void vTsk_testOnline(void *pvParams) {
    struct test_tsk *pHndl = pvParams;
    if (!pHndl) {
        vTaskSuspend(NULL);
    }

    gpio_set_mode(PIN_LED1, GPIO_MODE_OUTPUT);
    gpio_set_mode(PIN_LED2, GPIO_MODE_OUTPUT);
    gpio_set_mode(PIN_LED3, GPIO_MODE_OUTPUT);
    uint8_t leds = 1U;
    vTaskDelay(1000);
    for (;;) {
        leds ^= 0x3U;
        gpio_write(PIN_LED1, leds & 0x1U);
        gpio_write(PIN_LED2, leds & 0x2U);
        struct systime t;
        systime_fromTicks(xTaskGetTickCount(), &t);
        printf("Time: %s\n", t.str);
        vTaskDelay(pHndl->delay_ticks);
    }
}
