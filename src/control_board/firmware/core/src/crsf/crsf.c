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

#include "crsf/crsf_types.h"
#include "memory.h"
#include "pin_cfg.h"

#define FORCEPACKED

void vCRSF_Hndl_tsk(void* pvParams);
eCRSFError _send_packet(Serial_t* pSerial,
                        uint8_t len,
                        enum eCRSFMsgId type,
                        uint8_t* pData);
eCRSFError _recv_packet(
    Serial_t* pSerial, uint8_t addr, uint8_t len, uint8_t type, void* pData);
uint8_t crc8(const uint8_t* ptr, uint8_t len);

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

    return eCSRFOK;
}

eCRSFError crsf_write_rc(CRSF_t* pHndl, crsf_rc_t* pChannels) {
    if (!pHndl || !pChannels)
        return eCRSFNULL;
    if (pHndl->state != eCSRFOK)
        return pHndl->state;
    
    crsf_rc_t cin;

    // Sanitize inputs
    for(int i = 0; i < CRSF_N_CHANNELS; i++){
        uint16_t chnl = pChannels->channel[i];
        chnl = chnl > CRSF_CHANNEL_MAX ? CRSF_CHANNEL_MAX : chnl;
        chnl = chnl < CRSF_CHANNEL_MIN ? CRSF_CHANNEL_MIN : chnl;
        chnl = (uint16_t)(172 + (uint32_t)((uint32_t)chnl - 1000) * (1811 - 172) / (2000 - 1000));
        cin.channel[i] = chnl;
    }

    crsf_rcp_t msg;
    msg.chan0 = cin.channel[0];
    msg.chan1 = cin.channel[1];
    msg.chan2 = cin.channel[2];
    msg.chan3 = cin.channel[3];
    msg.chan4 = cin.channel[4];
    msg.chan5 = cin.channel[5];
    msg.chan6 = cin.channel[6];
    msg.chan7 = cin.channel[7];
    msg.chan8 = cin.channel[8];
    msg.chan9 = cin.channel[9];
    msg.chan10 = cin.channel[10];
    msg.chan11 = cin.channel[11];
    msg.chan12 = cin.channel[12];
    msg.chan13 = cin.channel[13];
    msg.chan14 = cin.channel[14];
    msg.chan15 = cin.channel[15];

    return _send_packet(pHndl->pSerial, CRSF_CHANNEL_BYTES, CRSFMsgRC, (void*)&msg);
}

eCRSFError crsf_read_gps(CRSF_t* pHndl, crsf_gps_t* pGPS) {
    if (!pHndl)
        return eCRSFNULL;
    if (!pGPS)
        return eCRSFNULL;
    if (pHndl->state != eCSRFOK)
        return pHndl->state;

    // Attempt to take the read semaphore from the buffering task
    if (xSemaphoreTake(pHndl->tx_hndl, CRSF_WAIT_TICKS) == pdTRUE) {
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

eCRSFError _send_packet(Serial_t* pSerial,
                        uint8_t len,
                        enum eCRSFMsgId type,
                        uint8_t * pData) {
    crsf_msg_t msg;
    msg.addr = 0xC8;
    msg.length = len + 2; // type + payload + crc
    msg.type = (uint8_t) type;
    uint8_t i = 0;
    for(; i < len; i++){
        msg.pyld[i] = pData[i];
        // msg.pyld[i] = 30;
    }

    uint8_t crc = crc8(&msg.type, len+1);
    msg.pyld[i] = crc;

    eSerialError e;
    e = serial_write_locked(
        pSerial, (void*) &msg, msg.length+2, 10, CRSF_SERIAL_LOCK);
    return (eCRSFError) e;
}

eCRSFError _recv_packet(
    Serial_t* pSerial, uint8_t addr, uint8_t len, uint8_t type, void* pData) {
}

unsigned char crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83,
    0xD7, 0x02, 0xA8, 0x7D, 0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
    0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F, 0xA4, 0x71, 0xDB, 0x0E,
    0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75,
    0x21, 0xF4, 0x5E, 0x8B, 0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
    0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0, 0xCF, 0x1A, 0xB0, 0x65,
    0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA,
    0xEE, 0x3B, 0x91, 0x44, 0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
    0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16, 0xEF, 0x3A, 0x90, 0x45,
    0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E,
    0x6A, 0xBF, 0x15, 0xC0, 0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
    0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36, 0x19, 0xCC, 0x66, 0xB3,
    0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1,
    0xA5, 0x70, 0xDA, 0x0F, 0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
    0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D, 0xD6, 0x03, 0xA9, 0x7C,
    0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07,
    0x53, 0x86, 0x2C, 0xF9};

uint8_t crc8(const uint8_t* ptr, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = crc8tab[crc ^ *ptr++];
    }
    return crc;
}
