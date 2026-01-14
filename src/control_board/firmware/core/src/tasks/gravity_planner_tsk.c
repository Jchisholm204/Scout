/**
 * @file gravity_planner_tsk.c
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2025-10-17
 * @modified Last Modified: 2025-10-17
 *
 * @copyright Copyright (c) 2025
 */

#include "tasks/gravity_planner_tsk.h"

void vGPlanTsk(void *pvParams);

QueueHandle_t gplan_tsk_init(struct gplan_tsk *pHndl,
                             QueueHandle_t usb_rx_front,
                             QueueHandle_t usb_rx_vertical,
                             QueueHandle_t usb_tx_front,
                             QueueHandle_t usb_tx_vertical) {
    if (!pHndl) {
        return NULL;
    }
    // Initialize the usb queues
    pHndl->usb.rx_front = usb_rx_front;
    pHndl->usb.rx_vertical = usb_rx_vertical;
    pHndl->usb.tx_front = usb_tx_front;
    pHndl->usb.tx_vertical = usb_tx_vertical;

    // Setup the Collision Vector Output Queue
    pHndl->cv_tx.hndl = xQueueCreateStatic(GPLAN_CVTX_BUF_SIZE,
                                           sizeof(quat_t),
                                           (uint8_t *) pHndl->cv_tx.buf,
                                           &pHndl->cv_tx.static_queue);

    if (!pHndl->cv_tx.hndl) {
        return NULL;
    }

    // Setup the Gravity Planner Task
    pHndl->tsk.hndl = xTaskCreateStatic(vGPlanTsk,
                                        "gplan",
                                        configMINIMAL_STACK_SIZE,
                                        pHndl,
                                        3,
                                        pHndl->tsk.stack,
                                        &pHndl->tsk.static_tsk);

    if (!pHndl->tsk.hndl) {
        return NULL;
    }

    return pHndl->cv_tx.hndl;
}

void vGPlanTsk(void *pvParams) {
    struct gplan_tsk *pHndl = (struct gplan_tsk *) pvParams;

    for (;;) {
        struct udev_pkt_lidar ldrpkt = {0};
        sniprintf((char *) &ldrpkt, sizeof(ldrpkt), "HelloFromGPlan\0");
        printf((const char *) &ldrpkt);
        xQueueSendToBack(pHndl->usb.tx_front, &ldrpkt, 10);
        vTaskDelay(100);
    }
}
