/**
 * @file trace.c
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief 
 * @version 0.1
 * @date Created: 2026-01-14
 * @modified Last Modified: 2026-01-14
 *
 * @copyright Copyright (c) 2026
 */

#include "stm32f4xx.h"
#include "core_cm4.h"

void trace_setup(void) {
    // 1. Ensure GPIOB Clock is enabled (for PB3/SWO)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    // 2. Configure PB3 for AF0 (SWO)
    GPIOB->MODER &= ~(3U << (3 * 2));
    GPIOB->MODER |= (2U << (3 * 2));     // Alternate Function
    GPIOB->OSPEEDR |= (3U << (3 * 2));   // Very High Speed
    GPIOB->AFR[0] &= ~(0xFU << (3 * 4)); // AF0

    // 3. Unlock and Enable Debug/Trace
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // 4. Configure TPIU - Protocol: Async NRZ (UART)
    TPI->SPPR = 0x00000002;

    // 5. Calculate ACPR (Prescaler)
    // 168,000,000 / 2,000,000 = 84. ACPR = 84 - 1 = 83.
    TPI->ACPR = 83;

    // 6. Disable Formatter (Required for many open-source decoders)
    TPI->FFCR = 0x0;

    // 7. ITM Setup
    ITM->LAR = 0xC5ACCE55; // Unlock
    ITM->TCR = ITM_TCR_ITMENA_Msk | ITM_TCR_SYNCENA_Msk;
    ITM->TER = 0x1; // Enable Stimulus Port 0
}
