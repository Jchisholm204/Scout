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
#include "crsf/crsf.h"

#include "crsf/crsf_types.h"
#include "crsf/crsf_types_internal.h"
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

    se = serial_attach(pSerial, &pHndl->rx_hndl);
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

    crsf_rc_t msg = {
        1500, 1500, 1500, 1500,
        1500, 1500, 1500, 1500,
        1500, 1500, 1500, 1500,
        1500, 1500, 1500, 1500
    };

    uint8_t mbuf[CRSF_CHANNEL_BYTES] = {127};
    gpio_set_mode(PIN_LED1, GPIO_MODE_OUTPUT);

    for (;;) {
        gpio_toggle_pin(PIN_LED1);
        // msg.chan3 += 10;
        // if (msg.chan3 > 2000)
        //     msg.chan3 = 1100;
        msg.channel[2]+=1;
        if(msg.channel[2] >= 1800) msg.channel[2] = 1200;
        uint e = crsf_write_rc(pHndl, &msg);
        
        // uint e = _send_packet(pHndl->pSerial, 22, CRSFMsgRC, (uint8_t*)&msg);
        // printf("Hello %d e=%d\n", msg.channel[2], e);
        // serial_write_locked(pHndl->pSerial)
        vTaskDelay(50);
    }
}

