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

    return 0;
}

float normalize_crsf(uint16_t raw) {
    if (raw < CRSF_CHANNEL_MIN)
        raw = CRSF_CHANNEL_MIN;
    if (raw > CRSF_CHANNEL_MAX)
        raw = CRSF_CHANNEL_MAX;
    return ((float) raw - 992.0f) / 819.5f;
}

float normalize_ctrl(float val) {
    return (val + 1.0f) / 2.0f;
}

float unnormal_ctrl(float val) {
    if (val < 0.0f) {
        val = 0.0f;
    }
    if (val > 1.0f) {
        val = 1.0f;
    }
    return (val * 2.0f) - 1.0f;
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

    float err_last = 0;
    float integral = 0;

    const float psc_const = 0.05f;
    const float p_const = 0.0042f;
    const float a_const = 0.1000f;
    const float g_const = 0.6000f;
    const float i_const = 0.0035f;
    const float d_const = 0.0020f;
    const float f_const = 0.2819f;

    float p = 0;
    float i = 0;
    float d = 0;
    float err = 0;

    for (;;) {
        crsf_rc_t rc;
        crsf_read_rc(&pHndl->rc_crsf.crsf, &rc);

        float x = normalize_ctrl(normalize_crsf(rc.chan2));
        float y = normalize_ctrl(normalize_crsf(rc.chan1));
        float z = normalize_ctrl(normalize_crsf(rc.chan0));
        float w = normalize_ctrl(normalize_crsf(rc.chan3));

        ctrl_state_t ct;
        if (xQueueReceive(pHndl->col_rx, &ct, 0) == pdTRUE) {
            float dt = ((float) xTaskGetTickCount() - (float) last_wake_time) /
                       (float) configTICK_RATE_HZ;
            if (dt <= 0.000001f) {
                dt = 0.005f;
            }
            err = -ct.cv.z * psc_const;
            p = p_const * err;
            float ip = err * (dt);
            if (err < 0) {
                // p = p * g_const;
                ip = ip * g_const;
            }
            integral += ip * a_const;
            integral = integral < 0.0f   ? 0.0f
                       : integral > 1.0f ? 1.0f
                                         : integral;

            i = i_const * integral;
            d = d_const * (err - err_last) / dt;
            err_last = err;
        }

        // Apply PID inputs
        if (rc.chan6 == eModeAuto) {
            z = (p + i + d + f_const);
        } else if (rc.chan6 == eModeSemi) {
            float a = (z - 0.5f) * 0.05f;
            z = f_const;
            z += a;
        } else {
            integral = 0;
        }
        printf("Z: %2.5f P: %1.5f I: %1.5f -> %2.6f\n", err, p, i, z);

        struct udev_pkt_ctrl_rx pkt_rx = (struct udev_pkt_ctrl_rx) {0};
        pkt_rx.vel.x = unnormal_ctrl(x);
        pkt_rx.vel.y = unnormal_ctrl(y);
        pkt_rx.vel.z = unnormal_ctrl(z);
        pkt_rx.vel.w = unnormal_ctrl(w);
        (void) xQueueGenericSend(pHndl->usb.rx, &pkt_rx, 1, queueOVERWRITE);

        vTaskDelayUntil(&last_wake_time, 20);
    }
}
