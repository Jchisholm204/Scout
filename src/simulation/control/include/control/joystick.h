/**
 * @file joystick.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2026-01-09
 * @modified Last Modified: 2026-01-09
 *
 * @copyright Copyright (c) 2026
 */

#ifndef _JOYSTICK_H_
#define _JOYSTICK_H_

#include <linux/uinput.h>

// Stolen from FrSky Controller Mappings
enum axis {
    AXIS_THROTTLE = ABS_X,
    AXIS_YAW = ABS_RX,
    AXIS_PITCH = ABS_Z,
    AXIS_ROLL = ABS_Y,
};

#define AXIS_MAX 23767
#define AXIS_MIN (-AXIS_MAX)

struct joystick {
    int fd;
};

extern int joystick_init(struct joystick *joy);

extern int joystick_write(struct joystick *joy, enum axis axis, int value);

extern int joystick_close(struct joystick *joy);

extern int joystick_test(void);

#endif

