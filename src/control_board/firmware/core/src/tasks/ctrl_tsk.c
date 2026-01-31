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

// rc channel 6 from the remote - top left toggle
enum eCtrlMode {
    eModeManual = 172,
    eModeSemi = 992,
    eModeAuto = 1809,
};

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

ctrl_vec_t ctrl_run_controllers(struct ctrl_tsk *const pHndl) {
    (void) pHndl;
    TickType_t last_wake_time = xTaskGetTickCount();
    const double f_const = 0.2819;
    const double psc_const = 0.05;
    float pid_z = 0.0f;

    float pid_x = 0;
    float pid_y = 0;

    // Handle Controller Input
    crsf_rc_t rc;
    crsf_read_rc(&pHndl->rc_crsf.crsf, &rc);
    crsf_write_rc(&pHndl->fc_crsf.crsf, &rc);
    crsf_battery_t bat;
    crsf_read_battery(&pHndl->fc_crsf.crsf, &bat);
    crsf_write_battery(&pHndl->rc_crsf.crsf, &bat);

    float ct_x = crsf_normalize(rc.chan2);
    float ct_y = crsf_normalize(rc.chan1);
    // Normalize the throttle to a percentatge value
    float ct_z = (crsf_normalize(rc.chan0) + 1.0f) / 2.0f;
    float ct_w = crsf_normalize(rc.chan3);

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
        // printf("Z: %3.3f X: %2.2f Y: %2.2f\n", pid_z, pid_x, pid_y);
    }

    ctrl_vec_t ctrl_v;
    ctrl_v.z = pid_z;
    ctrl_v.x *= 0.5f;
    ctrl_v.y *= 0.5f;
    ctrl_v.w *= 0.5f;
    ctrl_v.x += pid_x;
    ctrl_v.y += pid_y;

    return ctrl_v;
}

ctrl_vec_t ctrl_run_manual(struct ctrl_tsk *const pHndl) {
    (void) pHndl;

    ctrl_vec_t cv = {0};

    return cv;
}

void vCtrlTsk(void *pvParams) {
    struct ctrl_tsk *const pHndl = pvParams;

    // Run the setup functions for each of the controllers.
    ctrl_setup_controllers(pHndl);

    TickType_t last_wake_time = xTaskGetTickCount();
    const double f_const = 0.2819;
    const double psc_const = 0.05;
    float pid_z = 0.0f;

    float pid_x = 0;
    float pid_y = 0;

    for (;;) {
        // Handle Controller Input
        crsf_rc_t rc;
        crsf_read_rc(&pHndl->rc_crsf.crsf, &rc);
        crsf_write_rc(&pHndl->fc_crsf.crsf, &rc);
        crsf_battery_t bat;
        crsf_read_battery(&pHndl->fc_crsf.crsf, &bat);
        crsf_write_battery(&pHndl->rc_crsf.crsf, &bat);

        float ct_x = crsf_normalize(rc.chan2);
        float ct_y = crsf_normalize(rc.chan1);
        // Normalize the throttle to a percentatge value
        float ct_z = (crsf_normalize(rc.chan0) + 1.0f) / 2.0f;
        float ct_w = crsf_normalize(rc.chan3);

        ctrl_state_t target_state = {0};

        // Mode Switch Case
        switch ((enum eCtrlMode) rc.chan6) {
        case eModeManual:
            ctrl_run_manual(pHndl, &target_state);
            break;
        case eModeSemi:
            ctrl_run_(pHndl, &target_state);
            ctrl_run_manual(pHndl, &target_state);
            break;
        case eModeAuto:
            break;
        }

        // Send out control outputs
        struct udev_pkt_ctrl_rx pkt_rx = (struct udev_pkt_ctrl_rx) {0};
        pkt_rx.vel.x = ct_x;
        pkt_rx.vel.y = ct_y;
        pkt_rx.vel.z = ((ct_z * 2.0f) - 1.0f);
        pkt_rx.vel.w = ct_w;
        (void) xQueueGenericSend(pHndl->usb.rx, &pkt_rx, 1, queueOVERWRITE);

        vTaskDelayUntil(&last_wake_time, 20);
    }
}
