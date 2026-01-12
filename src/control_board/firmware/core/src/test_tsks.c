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
#include "main.h"
#include <stdio.h>
#include "stm32f446xx.h"
#include "FreeRTOS.h"
#include "config/FreeRTOSConfig.h"
#include "task.h"
#include "drivers/serial.h"
#include "string.h"
#include "hal/hal_clock.h"
#include "hal/hal_gpio.h"
#include "config/pin_cfg.h"
#include "drivers/canbus.h"
#include "os/systime.h"

void vTsk_testOnline(void * pvParams){
    (void)pvParams;
    char* str = "Hello World from Serial 2\n";
    gpio_set_mode(PIN_LED1, GPIO_MODE_OUTPUT);
    gpio_set_mode(PIN_LED2, GPIO_MODE_OUTPUT);
    gpio_set_mode(PIN_LED3, GPIO_MODE_OUTPUT);
    uint8_t leds = 1U;
    // hal_can_init(CAN1, CAN_1000KBPS, true, PIN_CAN1_TX, PIN_CAN1_RX);
    // RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    vTaskDelay(100);
    for(;;){
        leds ^= 0x3U;
        gpio_write(PIN_LED1, leds & 0x1U);
        gpio_write(PIN_LED2, leds & 0x2U);
        struct systime t;
        systime_fromTicks(xTaskGetTickCount(), &t);
        printf("Time: %s\n", t.str);
        vTaskDelay(1000);
    }
}
