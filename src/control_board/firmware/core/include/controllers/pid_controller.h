/**
 * @file pid_controller.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2026-01-16
 * @modified Last Modified: 2026-01-16
 *
 * @copyright Copyright (c) 2026
 */

#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

struct pid_controller {
    double const_p, const_i, const_d, const_a, const_ff;
    double integral;
    double err_last;
    double min, max;
};

static inline void pidc_init(struct pid_controller *pC,
                             double p,
                             double i,
                             double d,
                             double min,
                             double max) {
    pC->const_p = p;
    pC->const_i = i;
    pC->const_d = d;
    pC->const_ff = 0.0;
    pC->const_a = 0.005;
    pC->min = min;
    pC->max = max;
    pC->integral = 0;
    pC->err_last = 0;
}

static inline void pidc_set_ff(struct pid_controller *pC, double ff){
    pC->const_ff = ff;
}

static inline void pidc_set_accel(struct pid_controller *pC, double acceleration){
    pC->const_a = acceleration;
}

static inline void pidc_reset(struct pid_controller *pC){
    pC->integral = 0;
    pC->err_last = 0;
}


static inline double pidc_calculate(struct pid_controller *pC,
                                    double target,
                                    double input,
                                    double dt) {
    if(dt <= 0){
        dt = 0.0005;
    }
    double err = input - target;
    double p = err * pC->const_p;
    pC->integral += err * pC->const_a * dt;
    double i = pC->integral * pC->const_i;
    double d = ((err - pC->err_last) / dt) * pC->const_d;
    pC->err_last = err;
    double out = (p + i + d + pC->const_ff);
    if(out < pC->min)
        out = pC->min;
    if(out > pC->max)
        out = pC->max;
    return out;
}

#endif
