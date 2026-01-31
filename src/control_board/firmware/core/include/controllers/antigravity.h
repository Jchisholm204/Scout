/**
 * @file antigravity.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2026-01-16
 * @modified Last Modified: 2026-01-16
 *
 * @copyright Copyright (c) 2026
 */

#ifndef _ANTIGRAVITY_H_
#define _ANTIGRAVITY_H_
#include "drone_defs.h"

struct antigravity_controller {
    double base_throttle;
    double max_throttle;
    double current_throttle;
    ctrl_state_t last;
};

static inline void antigrav_init(struct antigravity_controller *pHndl,
                                 double base_throttle,
                                 double max_throttle) {
    pHndl->base_throttle = base_throttle;
    pHndl->current_throttle = base_throttle;
    pHndl->max_throttle = max_throttle;
    pHndl->last.cv = (ctrl_vec_t) {{0, 0, 0, 0}};
    pHndl->last.ceil_distance = 0.0;
    pHndl->last.ground_distance = 0.0;
}

static inline void antigrav_reset(struct antigravity_controller *pHndl) {
}

static inline ctrl_state_t antigrav_run(struct antigravity_controller *pHndl,
                                           ctrl_state_t *pCs) {
}

#endif
