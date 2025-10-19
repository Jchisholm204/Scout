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
#include "usb_packet.h"
#include "usb_interface.h"

// Task incldues
#include "crsf/crsf.h"

// Task Information Structures

CRSF_t tsk_crsf;

// Initialize all system Interfaces
void Init(void) {
    // Init USB Interface
    usbi_init();

    // Initialize UART
    serial_init(&Serial5, /*baud*/ CRSF_BAUD, PIN_UART5_RX, PIN_UART5_TX);

    /**
     * Initialize System Tasks...
     * All tasks should be initialized as static
     * Tasks can be initialized dynamically, but may crash the system if they
     * overflow the system memory (128Kb for the STM32f446)
     */
    crsf_init(&tsk_crsf, &Serial3, PIN_USART3_RX, PIN_USART3_TX);
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
