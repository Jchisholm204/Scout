/**
 * @file main_ctrl_tsk.c
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2026-01-12
 * @modified Last Modified: 2026-01-12
 *
 * @copyright Copyright (c) 2026
 */

#include "tasks/ctrl_tsk.h"

#include "drone_defs.h"
#include "queue.h"
#include "usb_cb_defs.h"
#include "usb_packet.h"

void vCtrlTsk(void *pvParams);
void vCtrlMonTsk(void *pvParams);

int ctrl_tsk_init(struct ctrl_tsk *pHndl,
                  Serial_t *rc_serial,
                  Serial_t *fc_serial,
                  QueueHandle_t usb_rx,
                  QueueHandle_t usb_tx,
                  QueueHandle_t col_rx) {

    pHndl->tsk.hndl = xTaskCreateStatic(vCtrlTsk,
                                        "ctrl_tsk",
                                        CTRL_TSK_STACK_SIZE,
                                        pHndl,
                                        configMAX_PRIORITIES - 2,
                                        pHndl->tsk.stack,
                                        &pHndl->tsk.static_task);

    pHndl->mon_tsk.hndl = xTaskCreateStatic(vCtrlMonTsk,
                                            "ctrl_mon_tsk",
                                            CTRL_TSK_STACK_SIZE,
                                            pHndl,
                                            configMAX_PRIORITIES - 3,
                                            pHndl->mon_tsk.stack,
                                            &pHndl->mon_tsk.static_task);

    pHndl->rc_crsf.buf_hndl =
        serial_create_write_buffer(rc_serial,
                                   configMINIMAL_STACK_SIZE,
                                   1,
                                   pHndl->rc_crsf.storage_area,
                                   &pHndl->rc_crsf.stream_buffer);

    StreamBufferHandle_t rx_hndl =
        crsf_init(&pHndl->rc_crsf.crsf, pHndl->rc_crsf.buf_hndl);

    if (!rx_hndl) {
        printf("CRSF INI Fail\n");
    }

    serial_attach(rc_serial, rx_hndl);

    pHndl->fc_crsf.buf_hndl =
        serial_create_write_buffer(fc_serial,
                                   configMINIMAL_STACK_SIZE,
                                   1,
                                   pHndl->fc_crsf.storage_area,
                                   &pHndl->fc_crsf.stream_buffer);

    rx_hndl = crsf_init(&pHndl->fc_crsf.crsf, pHndl->fc_crsf.buf_hndl);

    if (!rx_hndl) {
        printf("CRSF INI Fail\n");
    }

    serial_attach(fc_serial, rx_hndl);

    pHndl->usb.tx = usb_tx;
    pHndl->usb.rx = usb_rx;
    pHndl->col_rx = col_rx;

    return 0;
}

int ctrl_setup_controllers(struct ctrl_tsk *const pHndl) {

    // Hover/Z PID
    pidc_init(&pHndl->pid_z, 0.05, 0.0035, 0.05, 0, 0.6);
    pidc_set_accel(&pHndl->pid_z, 0.1);
    // Hover Constant
    pidc_set_ff(&pHndl->pid_z, 0.2819);

    const double p_xy = 0.002;
    const double d_xy = 0.0006;
    pidc_init(&pHndl->pid_x, p_xy, 0, d_xy, -0.4, 0.4);
    pidc_init(&pHndl->pid_y, p_xy, 0, d_xy, -0.4, 0.4);

    // antigrav_init(&pHndl->antigrav, 0.05, 0.5);

    return 0;
}

int ctrl_reset_controllers(struct ctrl_tsk *const pHndl) {
    // Reset the PID controllers
    pidc_reset(&pHndl->pid_z);
    pidc_reset(&pHndl->pid_x);
    pidc_reset(&pHndl->pid_y);

    // Reset the antigravity controller
    // antigrav_reset(&pHndl->antigrav);
    return 0;
}

ctrl_vec_t ctrl_run_controllers(struct ctrl_tsk *const pHndl,
                                ctrl_vec_t cv_in,
                                ctrl_vec_t cv_colsn) {
    TickType_t last_wake_time = xTaskGetTickCount();
    static double pid_z = 0;
    static double pid_y = 0;
    static double pid_x = 0;

    // Handle Lidar control input
    double dt = ((double) xTaskGetTickCount() - (double) last_wake_time) /
                (double) configTICK_RATE_HZ;
    if (dt <= 0.000001) {
        dt = 0.005;
    }
    pid_z = pidc_calculate(&pHndl->pid_z, 0, (double) -ctrl_vec_normal(cv_colsn).z, dt);
    pid_x = pidc_calculate(&pHndl->pid_x, 0, (double) -cv_colsn.x, dt);
    pid_y = pidc_calculate(&pHndl->pid_y, 0, (double) cv_colsn.y, dt);

    ctrl_vec_t ctrl_v = cv_in;
    ctrl_v.z = pid_z;
    ctrl_v.x *= 0.5;
    ctrl_v.y *= 0.5;
    ctrl_v.w *= 0.5;
    ctrl_v.x += pid_x;
    ctrl_v.y += pid_y;

    // ctrl_v = antigrav_run(&pHndl->antigrav, ctrl_v, cv_colsn);

    return ctrl_v;
}

ctrl_vec_t ctrl_run_manual(struct ctrl_tsk *const pHndl) {
    (void) pHndl;

    ctrl_vec_t cv = {0};
    crsf_rc_t rc;
    crsf_read_rc(&pHndl->rc_crsf.crsf, &rc);

    // TAER Mapping
    cv.x = (double) crsf_normalize(rc.chan2);
    cv.y = (double) crsf_normalize(rc.chan1);
    cv.z = (double) (crsf_normalize(rc.chan0) + 1.0) / 2.0;
    cv.w = (double) crsf_normalize(rc.chan3);

    return cv;
}

void vCtrlTsk(void *pvParams) {
    struct ctrl_tsk *const pHndl = pvParams;

    // Run the setup functions for each of the controllers.
    ctrl_setup_controllers(pHndl);

    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t last_jetson_time = xTaskGetTickCount();
    TickType_t last_collision_time = xTaskGetTickCount();

    pHndl->mode = eModeRC;

    for (;;) {
        // Ensure a consistent sample time delay
        vTaskDelayUntil(&last_wake_time, 5);

        // Read input data from the controller (primary source of truth)
        crsf_rc_t rc;
        crsf_read_rc(&pHndl->rc_crsf.crsf, &rc);

        // Transmit latest drone updates to the controller
        crsf_battery_t bat;
        crsf_read_battery(&pHndl->fc_crsf.crsf, &bat);
        crsf_write_battery(&pHndl->rc_crsf.crsf, &bat);

        // Check the arming condition
        if (rc.chan4 < CRSF_CHANNEL_ZERO ||
            pHndl->rc_crsf.crsf.state == eCRSFTimeout) {
            pHndl->mode = eModeDisabled;
        }
        // Control switch to init mode when arming condition is met
        else if (pHndl->mode == eModeDisabled) {
            pHndl->mode = eModeInit;
        }
        // Control Mode Switching
        else if (pHndl->mode != eModeInit) {
            // Decode operating parameters selections
            switch (rc.chan6) {
            case CRSF_CHANNEL_MIN:
                pHndl->mode = eModeRC;
                break;
            case CRSF_CHANNEL_ZERO:
                pHndl->mode = eModeRCAuto;
                break;
            case CRSF_CHANNEL_MAX:
                pHndl->mode = eModeAuto;
                break;
            default:
                break;
            }
            if (xTaskGetTickCount() > (last_jetson_time + 100) &&
                pHndl->mode == eModeAuto) {
                pHndl->mode = eModeStalled;
            }
        } else if (pHndl->mode == eModeInit) {
            // if (antigrav_checkstate(&pHndl->antigrav) == eAntigravStateNormal)
                pHndl->mode = eModeStalled;
        }

        // Read input data from the Jetson
        static ctrl_vec_t cv_jetson = {0};
        struct udev_pkt_ctrl_tx udev_ctrl;
        if (xQueueReceive(pHndl->usb.tx, &udev_ctrl, 0) == pdTRUE) {
            last_jetson_time = xTaskGetTickCount();
            cv_jetson.x = (double) udev_ctrl.vel.x;
            cv_jetson.y = (double) udev_ctrl.vel.y;
            cv_jetson.z = (double) udev_ctrl.vel.z;
            cv_jetson.w = (double) udev_ctrl.vel.w;
        }

        static ctrl_state_t cs_collision = {0};
        if (xQueueReceive(pHndl->col_rx, &cs_collision, 0) == pdTRUE) {
            last_collision_time = xTaskGetTickCount();
        }

        // Run controllers to get output control vector
        ctrl_vec_t cv_final = {0};
        switch (pHndl->mode) {
        case eModeInit:
            // cv_final = antigrav_liftoff(&pHndl->antigrav, cs_collision.cv);
            break;
        case eModeRC:
            cv_final = ctrl_run_manual(pHndl);
            break;
        case eModeStalled:
            cv_final = ctrl_run_controllers(pHndl,
                                            (ctrl_vec_t){0},
                                            cs_collision.cv);
            break;
        case eModeRCAuto:
            cv_final = ctrl_run_controllers(pHndl,
                                            ctrl_run_manual(pHndl),
                                            cs_collision.cv);
            break;
        case eModeAuto:
            cv_final = ctrl_run_controllers(pHndl, cv_jetson, cs_collision.cv);
            break;
        case eModeDisabled:
        case eModeFault:
            ctrl_reset_controllers(pHndl);
            break;
        }

        // Send out control outputs
        cv_final.z = (cv_final.z * 2.0) - 1.0;

        // USB Control Output
        struct udev_pkt_ctrl_rx pkt_rx = (struct udev_pkt_ctrl_rx) {0};
        for (int i = 0; i < 4; i++){
            pkt_rx.vel.data[i] = (float) cv_final.data[i];
            pkt_rx.cv.data[i] = (float) cs_collision.cv.data[i];
        }
        (void) xQueueGenericSend(pHndl->usb.rx, &pkt_rx, 1, queueOVERWRITE);

        // CRSF Output to Flight Controller
        crsf_rc_t fc_out = {0};
        // AETR Mappings
        fc_out.chan0 = crsf_unnormal(cv_final.x);
        fc_out.chan1 = crsf_unnormal(cv_final.y);
        fc_out.chan2 = crsf_unnormal(cv_final.z);
        fc_out.chan3 = crsf_unnormal(cv_final.w);
        fc_out.chan4 = rc.chan4;
        crsf_write_rc(&pHndl->fc_crsf.crsf, &fc_out);
    }
}

void vCtrlMonTsk(void *pvParams) {
    struct ctrl_tsk *const pHndl = pvParams;
    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;) {
        // Ensure a consistent sample time delay
        vTaskDelayUntil(&last_wake_time, 500);
        printf("Ctrl Mode: ");
        switch (pHndl->mode) {
        case eModeDisabled:
            printf("Disabled\n");
            break;
        case eModeInit:
            printf("Init\n");
            break;
        case eModeRC:
            printf("Remote Control\n");
            break;
        case eModeStalled:
            printf("Stalled\n");
            break;
        case eModeRCAuto:
            printf("RC Automatic\n");
            break;
        case eModeAuto:
            printf("Automatic\n");
            break;
        case eModeFault:
            printf("Fault\n");
            break;
        }
    }
}
