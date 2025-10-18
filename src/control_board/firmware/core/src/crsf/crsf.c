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

#include "crsf/crsf.h"
#include "memory.h"

void vCRSF_Hndl_tsk(void* pvParams);
eCRSFError _send_packet(
    Serial_t* pSerial, uint8_t addr, uint8_t len, uint8_t type, void* pData);
eCRSFError _recv_packet(
    Serial_t* pSerial, uint8_t addr, uint8_t len, uint8_t type, void* pData);

eCRSFError crsf_init(CRSF_t* pHndl, Serial_t* pSerial, pin_t srx, pin_t stx) {
    if (!pHndl)
        return eCRSFNULL;
    if (!pSerial)
        return eCRSFNULL;

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

    if (!pHndl->rx_hndl)
        return eCRSFNoPkt;

    // Setup the read semaphore
    pHndl->tx_hndl = xSemaphoreCreateMutexStatic(&pHndl->static_tx_semphr);
    if (!pHndl->tx_hndl)
        return eCRSFSemFail;

    // Setup the CRSF rx task
    pHndl->tsk_hndl = xTaskCreateStatic(vCRSF_Hndl_tsk,
                                        "CRSF",
                                        configMINIMAL_STACK_SIZE,
                                        (void*) pHndl,
                                        configMAX_PRIORITIES - 2,
                                        pHndl->tsk_stack,
                                        &pHndl->tsk_buf);
    if (!pHndl->tsk_hndl)
        return eCRSFTskCreateFail;

    se = serial_attach(pSerial, &pHndl->rx_hndl);
    if (se != eSerialOK)
        return eCRSFSerialFail;

    return eCSRFOK;
}

eCRSFError crsf_write_rc(CRSF_t* pHndl, crsf_rc_t* pChannels) {
    if (!pHndl)
        return eCRSFNULL;
    if (!pChannels)
        return eCRSFNULL;
    if (pHndl->state != eCSRFOK)
        return pHndl->state;

    // Sanitize data
    for (int i = 0; i < CRSF_N_CHANNELS; i++) {
        uint16_t* chnl = &pChannels->channel[i];
        *chnl = *chnl > CRSF_CHANNEL_MAX ? CRSF_CHANNEL_MAX : *chnl;
        *chnl = *chnl < CRSF_CHANNEL_MIN ? CRSF_CHANNEL_MIN : *chnl;
    }
    return _send_packet(pHndl->pSerial, 0, 0, 0, &pChannels);
}

eCRSFError crsf_read_gps(CRSF_t* pHndl, crsf_gps_t* pGPS) {
    if(!pHndl) return eCRSFNULL;
    if(!pGPS) return eCRSFNULL;
    if(pHndl->state != eCSRFOK) return pHndl->state;

    // Attempt to take the read semaphore from the buffering task
    if(xSemaphoreTake(pHndl->tx_hndl, CRSF_WAIT_TICKS) == pdTRUE){
        memcpy(pGPS, &pHndl->pkt.gps, sizeof(crsf_gps_t));
        xSemaphoreGive(pHndl->tx_hndl);
        return eCSRFOK;
    }
    return eCRSFSemFail;
}

eCRSFError crsf_read_battery(CRSF_t* pHndl, crsf_battery_t* pBattery) {
}

eCRSFError crsf_read_attitude(CRSF_t* pHndl, crsf_attitude_t* pAttitude) {
}

eCRSFError crsf_read_mode(CRSF_t* pHndl, crsf_fcmode_t* pMode) {
}

void vCRSF_Hndl_tsk(void* pvParams) {
    if (!pvParams)
        return;
    CRSF_t* pHndl = (CRSF_t*) pvParams;

    for (;;) {
        printf("Hello\n");
        vTaskDelay(500);
    }
}

eCRSFError _send_packet(
    Serial_t* pSerial, uint8_t addr, uint8_t len, uint8_t type, void* pData) {
}

eCRSFError _recv_packet(
    Serial_t* pSerial, uint8_t addr, uint8_t len, uint8_t type, void* pData) {
}
