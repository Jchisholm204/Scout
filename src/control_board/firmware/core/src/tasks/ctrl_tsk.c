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
        xStreamBufferCreateStatic(configMINIMAL_STACK_SIZE,
                                  1,
                                  pHndl->rc_crsf.storage_area,
                                  &pHndl->rc_crsf.stream_buffer);

    StreamBufferHandle_t rx_hndl =
        crsf_init(&pHndl->rc_crsf.crsf, pHndl->rc_crsf.buf_hndl);

    if (!rx_hndl) {
        printf("CRSF INI Fail\n");
    }

    serial_attach(rc_serial, rx_hndl);

    pHndl->usb.tx = usb_tx;
    pHndl->usb.rx = usb_rx;
    pHndl->col_rx = col_rx;

    const double p_const = 0.0042;
    const double a_const = 0.1000;
    // const double g_const = 0.6000;
    const double i_const = 0.0035;
    const double d_const = 0.0020;
    const double f_const = 0.2819;

    pidc_init(&pHndl->pid_z, p_const, i_const, d_const, 0, 0.5);
    pidc_set_accel(&pHndl->pid_z, a_const);
    pidc_set_ff(&pHndl->pid_z, f_const);

    antigrav_init(&pHndl->antigrav, 0.05, 0.5);

    return 0;
}

// rc channel 6 from the remote - top left toggle
enum eCtrlMode {
    eModeManual = 172,
    eModeSemi = 992,
    eModeAuto = 1809,
};

void vCtrlTsk(void *pvParams) {
    struct ctrl_tsk *pHndl = pvParams;

    TickType_t last_wake_time = xTaskGetTickCount();
    const double f_const = 0.2819;
    const double psc_const = 0.05;
    float pid_z = 0.0f;

    for (;;) {
        // Handle Controller Input
        crsf_rc_t rc;
        crsf_read_rc(&pHndl->rc_crsf.crsf, &rc);

        float ct_x = crsf_normalize(rc.chan2);
        float ct_y = crsf_normalize(rc.chan1);
        float ct_z = (crsf_normalize(rc.chan0) + 1.0f) / 2.0f;
        float ct_w = crsf_normalize(rc.chan3);

        // Handle Lidar control input
        ctrl_state_t ct;
        if (xQueueReceive(pHndl->col_rx, &ct, 0) == pdTRUE) {
            double dt =
                ((double) xTaskGetTickCount() - (double) last_wake_time) /
                (double) configTICK_RATE_HZ;
            if (dt <= 0.000001) {
                dt = 0.005;
            }
            pid_z = (float) pidc_calculate(
                &pHndl->pid_z, 0, (double) -ct.cv.z * psc_const, dt);
            printf("Z: %3.3f C: %2.2f G: %2.2f\n",
                   ct.cv.z,
                   ct.ceil_distance,
                   ct.ground_distance);
        }

        // Mode Switch Case
        switch ((enum eCtrlMode) rc.chan6) {
        case eModeManual:
            pidc_reset(&pHndl->pid_z);
            break;
        case eModeSemi:
            ct_z = (float) f_const + (ct_z - 0.5f) * 0.05f;
            break;
        case eModeAuto:
            ct_z = pid_z;
            ct_x *= 0.25f;
            ct_y *= 0.25f;
            ct_w *= 0.25f;
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
