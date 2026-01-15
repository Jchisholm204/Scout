/**
 * @file main.c
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief ELEC 498 Capstone - Central Control Board
 * @version 0.2
 * @date Created: 2025-1-19
 * @modified Last Modified: 2026-01-12
 *
 * @copyright Copyright (c) 2025
 */

#include "main.h"

#include "FreeRTOS.h"
#include "config/FreeRTOSConfig.h"
#include "config/pin_cfg.h"
#include "os/syscalls.h"
#include "os/systime.h"
#include "stm32f446xx.h"
#include "task.h"

#include <memory.h>
#include <stdio.h>
#include <string.h>

// Driver Includes
#include "drivers/serial.h"

// USB Includes
#include "usb/usb_interface.h"

// Task Includes
#include "tasks/ctrl_tsk.h"
#include "tasks/gravity_planner_tsk.h"
#include "tasks/test_tsks.h"

// Task Structures
struct ctrl_tsk ctrl_tsk;
struct gplan_tsk gplan_tsk;
struct test_tsk test_tsk;

// Initialize all system Interfaces
void Init(void) {
    // Initialize System Clock
    hal_clock_init();

    // Init USB Interface
    struct usbi *usbi = usbi_init();

    // Initialize UART
    Serial_t *Serial3 =
        serial_init(eSerial3, /*baud*/ 115200, PIN_USART3_RX, PIN_USART3_TX);
    Serial_t *Serial2 =
        serial_init(eSerial2, /*baud*/ CRSF_BAUD, PIN_USART2_RX, PIN_USART2_TX);

    // Register Serial Port 3 as STDIO
    // (Use this serial port for printf)
    register_stdio(Serial3);

    /**
     * Initialize System Tasks...
     * All tasks should be initialized as static
     * Tasks can be initialized dynamically, but may crash the system if they
     * overflow the system memory (128Kb for the STM32f446)
     */
    test_tsk_init(&test_tsk, 1000);
    QueueHandle_t cv_qh =
        gplan_tsk_init(&gplan_tsk, usbi->lidar_rx, usbi->lidar_tx);
    ctrl_tsk_init(&ctrl_tsk, Serial2, usbi->ctrl_rx, usbi->ctrl_tx);

    return;
}

int main(void) {
    // Call the init function
    Init();

    // Required ..?
    NVIC_SetPriorityGrouping(3);

    // Start Scheduler: Runs tasks initialized above
    vTaskStartScheduler();

    // System Main loop (Should never run -> Scheduler runs infinitely)
    for (;;) {
        asm("nop");
        // gpio_write(PIN_LED2, true);
    }
    return 0;
}
