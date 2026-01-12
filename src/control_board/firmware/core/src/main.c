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
#include "stm32f446xx.h"
#include "os/systime.h"
#include "task.h"
#include "usb/usb_interface.h"
#include "usb_cb_defs.h"

#include <memory.h>
#include <stdio.h>
#include <string.h>

// Task incldues
#include "protocols/crsf/crsf.h"

// Task Information Structures
CRSF_t tsk_crsf;


// Initialize all system Interfaces
void Init(void) {
    // Initialize System Clock
    hal_clock_init();
    // Init USB Interface
    struct usbi *usbi = usbi_init();

    // Verify system clock is stable after USB init
    hal_clock_init();

    // Initialize UART
    serial_init(&Serial3, /*baud*/ 9600, PIN_USART3_RX, PIN_USART3_TX);

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
