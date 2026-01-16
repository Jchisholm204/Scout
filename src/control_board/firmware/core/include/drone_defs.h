/**
 * @file drone_defs.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief Common definitions for the drone
 * @version 0.1
 * @date Created: 2025-10-17
 * @modified Last Modified: 2025-10-17
 *
 * @copyright Copyright (c) 2025
 */

#ifndef _DRONE_DEFS_H_
#define _DRONE_DEFS_H_

// Generic struct to describe drone position/velocity
typedef union {
    struct {
        float x, y, z, w;
    };
    float data[4];
} quat_t;

#endif
