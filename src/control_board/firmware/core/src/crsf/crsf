/**
 * @file crsf.c
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2025-10-14
 * @modified Last Modified: 2025-10-14
 *
 * @copyright Copyright (c) 2025
 */

#define CRSF_INTERNAL
#include "protocols/crsf/crsf.h"

#include "protocols/crsf/crsf_internal.h"
#include "protocols/crsf/crsf_types.h"
#include "protocols/crsf/crsf_types_internal.h"
#include "memory.h"
#include "pin_cfg.h"

void vCRSF_Hndl_tsk(void* pvParams);
extern eCRSFError _send_packet(Serial_t* pSerial,
                               uint8_t len,
                               enum eCRSFMsgId type,
                               uint8_t* pData);
extern eCRSFError _recv_packet(
    Serial_t* pSerial, uint8_t addr, uint8_t len, uint8_t type, void* pData);

eCRSFError crsf_init(CRSF_t* pHndl, Serial_t* pSerial, pin_t srx, pin_t stx) {
    if (!pHndl)
        return eCRSFNULL;
    if (!pSerial)
        return eCRSFNULL;
    pHndl->pSerial = pSerial;

    // Check serial state (must be uninit)
    if (pSerial->state != eSerialNoInit)
        return eCRSFInitFail;
    // Initialize the serial interface
    eSerialError se = serial_init(pSerial, CRSF_BAUD, srx, stx);
    if (se != eSerialOK)
        return eCRSFSerialFail;

    // Ensure nothing else can write to the CRSF serial port
    se = serial_lock(pSerial, CRSF_SERIAL_LOCK);
    if (se != eSerialOK)
        return eCRSFSerialFail;

    // Setup the internal stream buffer for Serial interrupts
    pHndl->rx_hndl = xStreamBufferCreateStatic(
        configMINIMAL_STACK_SIZE, 1, pHndl->rx_buf, &pHndl->rx_streamBuf);

    if (!pHndl->rx_hndl) {
        pHndl->state = eCRSFNoPkt;
        return pHndl->state;
    }

    // Setup the read semaphore
    pHndl->tx_hndl = xSemaphoreCreateMutexStatic(&pHndl->static_tx_semphr);
    if (!pHndl->tx_hndl) {
        pHndl->state = eCRSFSemFail;
        return pHndl->state;
    }

    // Setup the CRSF rx task
    pHndl->tsk_hndl = xTaskCreateStatic(vCRSF_Hndl_tsk,
                                        "CRSF",
                                        configMINIMAL_STACK_SIZE,
                                        (void*) pHndl,
                                        configMAX_PRIORITIES - 2,
                                        pHndl->tsk_stack,
                                        &pHndl->tsk_buf);
    if (!pHndl->tsk_hndl) {
        pHndl->state = eCRSFTskCreateFail;
        return pHndl->state;
    }

    // se = serial_attach(pSerial, &pHndl->rx_hndl);
    if (se != eSerialOK) {
        pHndl->state = eCRSFSerialFail;
        return pHndl->state;
    }

    return eCRSFOK;
}

void vCRSF_Hndl_tsk(void* pvParams) {
    if (!pvParams)
        return;
    CRSF_t* pHndl = (CRSF_t*) pvParams;

    crsf_rc_t msg_rc = {{{1500,
                        1500,
                        1500,
                        1500,
                        1500,
                        1500,
                        1500,
                        1500,
                        1500,
                        1500,
                        1500,
                        1500,
                        1500,
                        1500,
                        1500,
                        1500}}};

    gpio_set_mode(PIN_LED1, GPIO_MODE_OUTPUT);

    _crsf_msg_t rx_msg;
    uint8_t new_byte;
    uint8_t rx_buf[CRSF_DATA_MAXLEN] = {0};
    uint8_t rx_idx = 0;
    // serial_lock(&Serial5, 2);

    for (;;) {
        // MSG RX Logic
        crsf_msg_t valid_msg;

        // crsf_write_rc(pHndl, &msg_rc);

        // Attempt to pull the
        // while (xStreamBufferReceive(pHndl->rx_hndl, &new_byte, 1, 0) == 1) {
        //     rx_buf[rx_idx++] = new_byte;
        //     if (rx_idx == 1 && new_byte != CRSF_ADDR) {
        //         rx_idx = 0;
        //         continue;
        //     }
        //     if (rx_idx == 2 && (new_byte < 2 || new_byte > CRSF_DATA_MAXLEN)) {
        //         rx_idx = 0;
        //         continue;
        //     }
        //     if (rx_idx >= 2 && rx_idx == rx_buf[1] + 2) {
        //         eCRSFError e = _crsf_recv_packet((void*) &rx_buf, &valid_msg);
        //         if (e == eCRSFOK) {
        //             serial_write_locked(&Serial5, rx_buf, rx_buf[1] + 2, 10, 2);
        //             printf("Got Msg: %d\n", valid_msg.id);
        //             switch (valid_msg.id) {
        //             case CRSFMsgRC:
        //                 memcpy(&pHndl->pkt.rc,
        //                        &valid_msg.rc,
        //                        sizeof(crsf_rc_t));
        //                 break;
        //             case CRSFMsgLinkStat:
        //                 memcpy(&pHndl->pkt.link,
        //                        &valid_msg.link,
        //                        sizeof(crsf_rc_t));
        //                 break;
        //             case CRSFMsgBatt:
        //                 memcpy(&pHndl->pkt.batt,
        //                        &valid_msg.batt,
        //                        sizeof(crsf_rc_t));
        //                 break;
        //             case CRSFMsgFlightMode:
        //                 memcpy(&pHndl->pkt.mode,
        //                        &valid_msg.mode,
        //                        sizeof(crsf_rc_t));
        //             case CRSFMsgAtt:
        //                 memcpy(&pHndl->pkt.mode,
        //                        &valid_msg.mode,
        //                        sizeof(crsf_rc_t));
        //                 break;
        //             }
        //         } else {
        //             printf("Error: %d on type=0x%x id=0x%x\n",
        //                    e,
        //                    rx_buf[0],
        //                    valid_msg.id);
        //             char cbuf[5] = "Err ";
        //             cbuf[3] = (uint8_t) e;
        //             serial_write_locked(&Serial5, cbuf, 4, 10, 2);
        //         }
        //         rx_idx = 0;
        //     }
        // }

        printf("Hello\n");
        gpio_toggle_pin(PIN_LED1);
        vTaskDelay(500);
    }
}
