/**
 * @file main.c
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief ELEC 498 Capstone - Central Control Board
 * @version 0.1
 * @date Created: 2025-1-19
 * @modified Last Modified: 2025-10-14
 *
 * @copyright Copyright (c) 2025
 */

#include "main.h"

#include "FreeRTOS.h"
#include "config/FreeRTOSConfig.h"
#include "config/pin_cfg.h"
#include "drivers/canbus.h"
#include "drivers/serial.h"
#include "hal/hal_usb.h"
#include "stm32f446xx.h"
#include "systime.h"
#include "task.h"

#include <memory.h>
#include <stdio.h>
#include <string.h>

// USB Device Includes
#include "drivers/stusb/usb.h"
#include "test_tsks.h"
#include "usb_desc.h"
#include "usb_interface.h"

// Task incldues
#include "crsf/crsf.h"

// Task Information Structures

CRSF_t tsk_crsf;

void test(void *pvParams){
    (void)pvParams;
    gpio_set_mode(PIN_LED1, GPIO_MODE_OUTPUT);
    gpio_set_mode(PIN_LED2, GPIO_MODE_OUTPUT);
    gpio_set_mode(PIN_LED3, GPIO_MODE_OUTPUT);
    gpio_toggle_pin(PIN_LED2);
    for(;;){
        printf("Hello Uart\n");
        vTaskDelay(500);
        gpio_toggle_pin(PIN_LED1);
        gpio_toggle_pin(PIN_LED2);
        gpio_toggle_pin(PIN_LED3);
    }
}

// Initialize all system Interfaces
void Init(void) {
    // Init USB Interface
    hal_clock_init();
    usbi_init();
    hal_clock_init();

    // Initialize UART
    serial_init(&Serial3, /*baud*/ 9600, PIN_USART3_RX, PIN_USART3_TX);

    // xTaskCreate(test, "test", 100, NULL, 2, NULL);

    /**
     * Initialize System Tasks...
     * All tasks should be initialized as static
     * Tasks can be initialized dynamically, but may crash the system if they
     * overflow the system memory (128Kb for the STM32f446)
     */
    // crsf_init(&tsk_crsf, &Serial2, PIN_USART2_RX, PIN_USART2_TX);

    return;
}


int main(void) {
    // Call the init function
    Init();

    // Start Scheduler: Runs tasks initialized above
    vTaskStartScheduler();

    // System Main loop (Should never run -> Scheduler runs infinitely)
    for (;;) {
        asm("nop");
        // gpio_write(PIN_LED2, true);
    }
    return 0;
}
