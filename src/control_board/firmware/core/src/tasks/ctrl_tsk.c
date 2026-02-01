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
    const double p_const = 0.0042;
    const double a_const = 0.1000;
    // const double g_const = 0.6000;
    const double i_const = 0.0035;
    const double d_const = 0.0020;
    const double f_const = 0.2819;

    pidc_init(&pHndl->pid_z, p_const, i_const, d_const, 0, 0.5);
    pidc_set_accel(&pHndl->pid_z, a_const);
    pidc_set_ff(&pHndl->pid_z, f_const);

    const double p_xy = 0.00180;
    const double d_xy = 0.00025;
    pidc_init(&pHndl->pid_x, p_xy, 0, d_xy, -0.15, 0.15);
    pidc_init(&pHndl->pid_y, p_xy, 0, d_xy, -0.15, 0.15);

    antigrav_init(&pHndl->antigrav, 0.05, 0.5);

    return 0;
}

int ctrl_reset_controllers(struct ctrl_tsk *const pHndl) {
    // Reset the PID controllers
    pidc_reset(&pHndl->pid_z);
    pidc_reset(&pHndl->pid_x);
    pidc_reset(&pHndl->pid_y);

    // Reset the antigravity controller
    antigrav_reset(&pHndl->antigrav);
    return 0;
}

ctrl_vec_t ctrl_run_controllers(struct ctrl_tsk *const pHndl,
                                ctrl_vec_t cv_in) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const double f_const = 0.2819;
    const double psc_const = 0.05;
    static float pid_z = 0.0f;

    static float pid_x = 0;
    static float pid_y = 0;

    // Handle Lidar control input
    ctrl_state_t ct;
    if (xQueueReceive(pHndl->col_rx, &ct, 0) == pdTRUE) {
        double dt = ((double) xTaskGetTickCount() - (double) last_wake_time) /
                    (double) configTICK_RATE_HZ;
        if (dt <= 0.000001) {
            dt = 0.005;
        }
        pid_z = (float) pidc_calculate(
            &pHndl->pid_z, 0, (double) -ct.cv.z * psc_const, dt);
        pid_x = (float) pidc_calculate(&pHndl->pid_x, 0, (double) -ct.cv.x, dt);
        pid_y = (float) pidc_calculate(&pHndl->pid_y, 0, (double) ct.cv.y, dt);
    }

    static ctrl_vec_t ctrl_v;
    ctrl_v.z = pid_z;
    ctrl_v.x *= 0.5f;
    ctrl_v.y *= 0.5f;
    ctrl_v.w *= 0.5f;
    ctrl_v.x += pid_x;
    ctrl_v.y += pid_y;

    return ctrl_vec_combine(ctrl_v, cv_in, 0.5);
}

ctrl_vec_t ctrl_run_manual(struct ctrl_tsk *const pHndl) {
    (void) pHndl;

    ctrl_vec_t cv = {0};
    crsf_rc_t rc;
    crsf_read_rc(&pHndl->rc_crsf.crsf, &rc);

    // TAER Mapping
    cv.x = crsf_normalize(rc.chan2);
    cv.y = crsf_normalize(rc.chan1);
    cv.z = (crsf_normalize(rc.chan0) + 1.0f) / 2.0f;
    cv.w = crsf_normalize(rc.chan3);

    return cv;
}

void vCtrlTsk(void *pvParams) {
    struct ctrl_tsk *const pHndl = pvParams;

    // Run the setup functions for each of the controllers.
    ctrl_setup_controllers(pHndl);

    TickType_t last_wake_time = xTaskGetTickCount();
    TickType_t last_update_time = xTaskGetTickCount();
    TickType_t last_jetson_time = xTaskGetTickCount();

    enum eCBMode current_mode = eModeRC;

    for (;;) {
        // Ensure a consistent sample time delay
        vTaskDelayUntil(&last_wake_time, 20);

        if (xTaskGetTickCount() >= (last_update_time + 500)) {
            last_update_time = xTaskGetTickCount();
            printf("Ctrl Mode: ");
            switch (current_mode) {
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

        // Read input data from the controller (primary source of truth)
        crsf_rc_t rc;
        crsf_read_rc(&pHndl->rc_crsf.crsf, &rc);

        // Transmit latest drone updates to the controller
        crsf_battery_t bat;
        crsf_read_battery(&pHndl->fc_crsf.crsf, &bat);
        crsf_write_battery(&pHndl->rc_crsf.crsf, &bat);

        // Enable mode switching operations under these conditions
        if (current_mode != eModeDisabled && current_mode != eModeInit) {
            // Decode operating parameters selections
            switch (rc.chan6) {
            case CRSF_CHANNEL_MIN:
                current_mode = eModeRC;
                break;
            case CRSF_CHANNEL_ZERO:
                current_mode = eModeRCAuto;
                break;
            case CRSF_CHANNEL_MAX:
                current_mode = eModeAuto;
                break;
            default:
                break;
            }
        }

        // Check the arming condition
        if (rc.chan4 < CRSF_CHANNEL_ZERO) {
            current_mode = eModeDisabled;
        }
        // Control switch to init mode when arming condition is met
        else if (current_mode == eModeDisabled) {
            current_mode = eModeInit;
        }

        // Read input data from the Jetson
        static ctrl_vec_t cv_jetson = {0};
        struct udev_pkt_ctrl_tx udev_ctrl;
        if (xQueueReceive(pHndl->usb.tx, &udev_ctrl, 0) == pdTRUE) {
            last_jetson_time = xTaskGetTickCount();
            cv_jetson.x = udev_ctrl.vel.x;
            cv_jetson.y = udev_ctrl.vel.y;
            cv_jetson.z = udev_ctrl.vel.z;
            cv_jetson.w = udev_ctrl.vel.w;
        } else if (xTaskGetTickCount() > (last_jetson_time + 100) &&
                   current_mode == eModeAuto) {
            current_mode = eModeStalled;
        }

        // Run controllers to get output control vector
        ctrl_vec_t cv_final = {0};
        switch (current_mode) {
        case eModeInit:
            // if (ctrl_setup_controllers(pHndl)) {
            //     current_mode = eModeStalled;
            // }
            ctrl_setup_controllers(pHndl);
            current_mode = eModeStalled;
            break;
        case eModeRC:
            cv_final = ctrl_run_manual(pHndl);
            break;
        case eModeStalled:
        case eModeRCAuto:
            cv_final = ctrl_run_controllers(pHndl, ctrl_run_manual(pHndl));
            break;
        case eModeAuto:
            cv_final = ctrl_run_controllers(pHndl, cv_jetson);
            break;
        case eModeDisabled:
        case eModeFault:
            ctrl_reset_controllers(pHndl);
            break;
        }

        // Send out control outputs
        cv_final.z = (cv_final.z * 2.0f) - 1.0f;

        // USB Control Output
        struct udev_pkt_ctrl_rx pkt_rx = (struct udev_pkt_ctrl_rx) {0};
        for (int i = 0; i < 4; i++)
            pkt_rx.vel.data[i] = cv_final.data[i];
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
