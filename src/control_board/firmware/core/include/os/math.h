/**
 * @file math.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief 
 * @version 0.1
 * @date Created: 2026-02-01
 * @modified Last Modified: 2026-02-01
 *
 * @copyright Copyright (c) 2026
 */

#ifndef _ARM_MATH_H_
#define _ARM_MATH_H_

static inline float sqrtf(float val) {
    float res;
    __asm volatile("vsqrt.f32 %0, %1" : "=t"(res) : "t"(val));
    return res;
}

#endif
