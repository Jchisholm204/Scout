/**
 * @file joystick.cpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2026-01-09
 * @modified Last Modified: 2026-01-09
 *
 * @copyright Copyright (c) 2026
 */

#include "control/joystick.h"

#include <fcntl.h>
#include <linux/uinput.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

void emit(int fd, int type, int code, int val) {
    struct input_event ie;
    ie.type = type;
    ie.code = code;
    ie.value = val;
    ie.time.tv_sec = 0;
    ie.time.tv_usec = 0;
    write(fd, &ie, sizeof(struct input_event));
}

int joystick_init(struct joystick *joy) {
    if (!joy) {
        return -1;
    }
    joy->fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);

    if (!joy->fd) {
        return !joy->fd;
    }

    ioctl(joy->fd, UI_SET_EVBIT, EV_ABS);
    ioctl(joy->fd, UI_SET_EVBIT, EV_KEY);
    ioctl(joy->fd, UI_SET_KEYBIT, BTN_TRIGGER);

    int axes[] = {AXIS_THROTTLE, AXIS_YAW, AXIS_PITCH, AXIS_ROLL};

    for (int i = 0; i < 4; i++) {
        ioctl(joy->fd, UI_SET_ABSBIT, axes[i]);
    }

    struct uinput_abs_setup abs_setup;
    for (int i = 0; i < 4; i++) {
        memset(&abs_setup, 0, sizeof(abs_setup));
        abs_setup.code = axes[i];
        abs_setup.absinfo.minimum = AXIS_MIN;
        abs_setup.absinfo.maximum = AXIS_MAX;
        ioctl(joy->fd, UI_ABS_SETUP, &abs_setup);
    }

    struct uinput_setup usetup;
    memset(&usetup, 0, sizeof(usetup));
    usetup.id.bustype = BUS_USB;
    usetup.id.vendor = 0x1234; // Dummy ID
    usetup.id.product = 0x5678;
    strcpy(usetup.name, "Liftoff-Testbench-Controller");

    ioctl(joy->fd, UI_DEV_SETUP, &usetup);
    ioctl(joy->fd, UI_DEV_CREATE);

    return 0;
}

int joystick_write(struct joystick *joy, enum axis axis, int value) {
    if (!joy) {
        return -1;
    }

    emit(joy->fd, EV_ABS, (int) axis, value);
    // Sync is mandatory to "flush" the event
    emit(joy->fd, EV_SYN, SYN_REPORT, 0);
    return 0;
}

int joystick_close(struct joystick *joy) {
    ioctl(joy->fd, UI_DEV_DESTROY);
    return close(joy->fd);
}

int joystick_test(void) {
    printf("Joystick Test\n");

    struct joystick joy;
    joystick_init(&joy);

    printf("Virtual Joystick Created. Calibrate in Liftoff now...\n");
    int value = 0;
    while (1) {
        // Example: Sending a slow sine wave to the Roll axis (ABS_X)
        // In your real code, this data comes from your control algorithm
        joystick_write(&joy, AXIS_ROLL, value);
        joystick_write(&joy, AXIS_THROTTLE, AXIS_MAX - value);
        value++;
        if (value == AXIS_MAX) {
            value = AXIS_MIN;
        }
        usleep(10000); // 100Hz
    }

    joystick_close(&joy);

    return 0;
}
