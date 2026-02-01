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

enum eAntigravState {
    eAntigravStateReset,
    eAntigravStateLiftoff,
    eAntigravStateAutotune,
    eAntigravStateNormal,
};

struct antigravity_controller {
    double base_throttle;
    double max_throttle;
    double current_throttle;
    ctrl_vec_t last;
    enum eAntigravState state;
};

static inline void antigrav_init(struct antigravity_controller *const pHndl,
                                 double base_throttle,
                                 double max_throttle) {
    pHndl->base_throttle = base_throttle;
    pHndl->current_throttle = base_throttle;
    pHndl->max_throttle = max_throttle;
    pHndl->last = (ctrl_vec_t) {{0, 0, 0, 0}};
    pHndl->state = eAntigravStateReset;
}

static inline void antigrav_reset(struct antigravity_controller *const pHndl) {
    pHndl->state = eAntigravStateReset;
    pHndl->current_throttle = 0.0;
}

static inline ctrl_vec_t antigrav_liftoff(
    struct antigravity_controller *const pHndl, const ctrl_vec_t col_vec) {
    if (pHndl->state != eAntigravStateLiftoff) {
        pHndl->last = (ctrl_vec_t) {{0, 0, 0, 0}};
        pHndl->current_throttle = pHndl->base_throttle;
    }
    pHndl->state = eAntigravStateLiftoff;

    pHndl->current_throttle = col_vec.z + 0.28;
    // ctrl_vec_print("Liftoff", col_vec);
    ctrl_vec_t cv = {0};
    cv.z = pHndl->current_throttle;

    printf("Liftoff: z=%.2f t=%.4f\n", col_vec.z, pHndl->current_throttle);

    if(col_vec.z == 0){
        pHndl->state = eAntigravStateNormal;
    }

    return cv;
}

static inline enum eAntigravState antigrav_checkstate(
    struct antigravity_controller *const pHndl) {
    return pHndl->state;
}

static inline ctrl_vec_t antigrav_run(
    struct antigravity_controller *const pHndl,
    ctrl_vec_t cv_ctrl,
    ctrl_vec_t cv_col) {
    cv_ctrl.z = pHndl->current_throttle;
    return cv_ctrl;
}

#endif
