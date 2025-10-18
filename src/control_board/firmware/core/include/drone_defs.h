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

// Drone Propeller size in inches
#define DRONE_PROP_SIZE 9

// Drone Anti-Gravity PID values
#define DRONE_ANTIG_P 1
#define DRONE_ANTIG_I 0
#define DRONE_ANTIG_D 0

// Generic struct to describe drone position/velocity
typedef struct {
    float x, y, z, w;
}  drone_pos_t;

#endif
