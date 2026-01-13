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

void vCtrlTsk(void *pvParams);

int ctrl_tsk_init(struct ctrl_tsk *pHndl, Serial_t *pSerial) {
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

    StreamBufferHandle_t *rx_hndl = crsf_init(&pHndl->crsf, &pHndl->tx_hndl);

    if(!rx_hndl){
        printf("CRSF INI Fail\n");
    }

    serial_attach(pHndl->pSerial, rx_hndl);
    return 0;
}

void vCtrlTsk(void *pvParams) {
    struct ctrl_tsk *pHndl = pvParams;

    for (;;) {
        crsf_rc_t rc;
        crsf_read_rc(&pHndl->crsf, &rc);
        printf("%d: %d %d %d %d\n",
               pHndl->crsf.state,
               rc.chan0,
               rc.chan1,
               rc.chan2,
               rc.chan3);
        vTaskDelay(100);
    }
}
