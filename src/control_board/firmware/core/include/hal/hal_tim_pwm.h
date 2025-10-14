/**
 * @file hal_tim_pwm.h
 * @author Jacob Chisholm (Jchisholm204.github.io)
 * @brief HAL Library for STM32 Timer PWM
 * @version 0.1
 * @date 2025-01-24
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef _HAL_TIM_PWM_H_
#define _HAL_TIM_PWM_H_
#include <stm32f4xx.h>
#include "hal_gpio.h"

// ARR = Frequency 
// CCRx = Duty Cycle 
// PWM can be ran on a per channel basis (one PWM per OCx)
//  with PWM mode 1 (OCxM = 110 or OCxM = 111 for PWM mode 2)
//  The preload register must be enabled with the OCxPE bit in the CCMRx Register
//  and the autoreload bit (ARPE) must be set in the CR1 register
// All timers must be initialized by setting the UG bit in the EGR
//

enum eTimCh {
    eTimCh1,
    eTimCh2,
    eTimCh3,
    eTimCh4
};

static inline void hal_tim_pwm_init(TIM_TypeDef *pTIM, uint16_t prescaler, uint16_t arr) {
    if (pTIM == TIM2) SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM2EN);
    if (pTIM == TIM3) SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM3EN);
    if (pTIM == TIM4) SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM4EN);
    if (pTIM == TIM5) SET_BIT(RCC->APB1ENR, RCC_APB1ENR_TIM5EN);

    CLEAR_REG(pTIM->CR1);
    SET_BIT(pTIM->CR1, TIM_CR1_ARPE);  // Enable auto-reload preload
    CLEAR_REG(pTIM->CR2);
    CLEAR_REG(pTIM->SMCR);
    CLEAR_REG(pTIM->DIER);
    CLEAR_REG(pTIM->EGR);
    SET_BIT(pTIM->EGR, TIM_EGR_UG);    // Generate update event
    CLEAR_REG(pTIM->CCMR1);
    CLEAR_REG(pTIM->CCMR2);
    CLEAR_REG(pTIM->CCER);

    pTIM->PSC = prescaler; // Set prescaler
    pTIM->ARR = arr;       // Set auto-reload value

    SET_BIT(pTIM->CR1, TIM_CR1_CEN); // Enable the timer
}

static inline void hal_tim_pwm_configure_channel(TIM_TypeDef *pTIM, enum eTimCh ch) {
    switch (ch) {
        case eTimCh1:
            CLEAR_BIT(pTIM->CCMR1, TIM_CCMR1_OC1M);
            SET_BIT(pTIM->CCMR1, TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2); // PWM Mode 1
            SET_BIT(pTIM->CCMR1, TIM_CCMR1_OC1PE);                     // Enable preload
            SET_BIT(pTIM->CCER, TIM_CCER_CC1E);                       // Enable channel output
            break;
        case eTimCh2:
            CLEAR_BIT(pTIM->CCMR1, TIM_CCMR1_OC2M);
            SET_BIT(pTIM->CCMR1, TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);
            SET_BIT(pTIM->CCMR1, TIM_CCMR1_OC2PE);
            SET_BIT(pTIM->CCER, TIM_CCER_CC2E);
            break;
        case eTimCh3:
            CLEAR_BIT(pTIM->CCMR2, TIM_CCMR2_OC3M);
            SET_BIT(pTIM->CCMR2, TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2);
            SET_BIT(pTIM->CCMR2, TIM_CCMR2_OC3PE);
            SET_BIT(pTIM->CCER, TIM_CCER_CC3E);
            break;
        case eTimCh4:
            CLEAR_BIT(pTIM->CCMR2, TIM_CCMR2_OC4M);
            SET_BIT(pTIM->CCMR2, TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2);
            SET_BIT(pTIM->CCMR2, TIM_CCMR2_OC4PE);
            SET_BIT(pTIM->CCER, TIM_CCER_CC4E);
            break;
    }
}

static inline void hal_tim_pwm_set(TIM_TypeDef *pTIM, enum eTimCh ch, uint32_t preload) {
    switch (ch) {
        case eTimCh1:
            pTIM->CCR1 = preload;
            break;
        case eTimCh2:
            pTIM->CCR2 = preload;
            break;
        case eTimCh3:
            pTIM->CCR3 = preload;
            break;
        case eTimCh4:
            pTIM->CCR4 = preload;
            break;
    }
}

#endif
