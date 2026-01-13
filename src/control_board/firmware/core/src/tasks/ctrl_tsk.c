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

#include "queue.h"
#include "usb_packet.h"

void vCtrlTsk(void *pvParams);

int ctrl_tsk_init(struct ctrl_tsk *pHndl,
                  Serial_t *pSerial,
                  QueueHandle_t ctrl_rx,
                  QueueHandle_t ctrl_tx) {
    pHndl->pSerial = pSerial;
    pHndl->tsk_hndl = xTaskCreateStatic(vCtrlTsk,
                                        "ctrl_tsk",
                                        configMINIMAL_STACK_SIZE,
                                        pHndl,
                                        2,
                                        pHndl->tsk_stack,
                                        &pHndl->tsk_buf);

    pHndl->tx_hndl = xStreamBufferCreateStatic(
        configMINIMAL_STACK_SIZE, 1, pHndl->tx_buf, &pHndl->tx_streamBuf);

    // StreamBufferHandle_t *rx_hndl = crsf_init(&pHndl->crsf, &pHndl->tx_hndl);

    // if (!rx_hndl) {
    //     printf("CRSF INI Fail\n");
    // }
    //
    // serial_attach(pHndl->pSerial, rx_hndl);

    pHndl->ctrl_rx = ctrl_rx;
    pHndl->ctrl_tx = ctrl_tx;

    return 0;
}

float normalize_crsf(uint16_t raw) {
    if (raw < CRSF_CHANNEL_MIN)
        raw = CRSF_CHANNEL_MIN;
    if (raw < CRSF_CHANNEL_MAX)
        raw = CRSF_CHANNEL_MAX;
    return ((float) raw - 992.0f) / 819.5f;
}

void vCtrlTsk(void *pvParams) {
    struct ctrl_tsk *pHndl = pvParams;

    int transmissions = 0;

    for (;;) {
        crsf_rc_t rc;
        crsf_read_rc(&pHndl->crsf, &rc);
        printf("%d: %d %d %d %d\n",
               pHndl->crsf.state,
               rc.chan0,
               rc.chan1,
               rc.chan2,
               rc.chan3);

        struct udev_pkt_ctrl_rx pkt_rx = (struct udev_pkt_ctrl_rx) {
            {transmissions, 'E'}, 'H', 'E', 'L', '\n'};
        // pkt_rx.vel.x = normalize_crsf(rc.chan2);
        // pkt_rx.vel.y = normalize_crsf(rc.chan1);
        // pkt_rx.vel.z = normalize_crsf(rc.chan0);
        // pkt_rx.vel.z = normalize_crsf(rc.chan3);
        BaseType_t e;
        if (pHndl->ctrl_rx) {
            if ((e = xQueueGenericSend(
                     pHndl->ctrl_rx, &pkt_rx, 1, queueOVERWRITE)) != pdTRUE) {
                printf("QueueFail %ld %d\n", e, transmissions);
            } else {
                transmissions++;
                printf("Queue Sent %d\n", transmissions);
            }
        }

        vTaskDelay(500);
    }
}
