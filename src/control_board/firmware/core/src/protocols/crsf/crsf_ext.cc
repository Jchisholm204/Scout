/**
 * @file crsf_ext.c
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief External CRSF Methods
 * @version 0.1
 * @date Created: 2025-10-18
 * @modified Last Modified: 2025-10-18
 *
 * @copyright Copyright (c) 2025
 */

// Mandatory defines for all internal modules
#define CRSF_INTERNAL

#include "protocols/crsf/crsf.h"
#include "protocols/crsf/crsf_types.h"
#include "protocols/crsf/crsf_types_internal.h"

#include <string.h>

// send/recv methods from crsf_pkt
extern eCRSFError _crsf_send_packet(Serial_t* pSerial,
                               uint8_t len,
                               enum eCRSFMsgId type,
                               uint8_t* pData);
eCRSFError crsf_write_rc(CRSF_t* pHndl, crsf_rc_t* pChannels) {
    if (!pHndl || !pChannels)
        return eCRSFNULL;
    if (pHndl->state != eCRSFOK)
        return pHndl->state;

    // Create temp copy of data
    crsf_rc_t cin;

    // Sanitize inputs
    for (int i = 0; i < CRSF_N_CHANNELS; i++) {
        uint16_t chnl = pChannels->channel[i];
        chnl = chnl > CRSF_CHANNEL_MAX ? CRSF_CHANNEL_MAX : chnl;
        chnl = chnl < CRSF_CHANNEL_MIN ? CRSF_CHANNEL_MIN : chnl;
        chnl = (uint16_t) (172 + (uint32_t) ((uint32_t) chnl - 1000) *
                                     (1811 - 172) / (2000 - 1000));
        cin.channel[i] = chnl;
    }

    // Copy the data into the internal packed form
    _crsf_rc_t msg;
    msg.chan0 = (unsigned int)cin.channel[0] & 0x07FF;
    msg.chan1 = (unsigned int)cin.channel[1] & 0x07FF;
    msg.chan2 = (unsigned int)cin.channel[2] & 0x07FF;
    msg.chan3 = (unsigned int)cin.channel[3] & 0x07FF;
    msg.chan4 = (unsigned int)cin.channel[4] & 0x07FF;
    msg.chan5 = (unsigned int)cin.channel[5] & 0x07FF;
    msg.chan6 = (unsigned int)cin.channel[6] & 0x07FF;
    msg.chan7 = (unsigned int)cin.channel[7] & 0x07FF;
    msg.chan8 = (unsigned int)cin.channel[8] & 0x07FF;
    msg.chan9 = (unsigned int)cin.channel[9] & 0x07FF;
    msg.chan10 = (unsigned int)cin.channel[10] & 0x07FF;
    msg.chan11 = (unsigned int)cin.channel[11] & 0x07FF;
    msg.chan12 = (unsigned int)cin.channel[12] & 0x07FF;
    msg.chan13 = (unsigned int)cin.channel[13] & 0x07FF;
    msg.chan14 = (unsigned int)cin.channel[14] & 0x07FF;
    msg.chan15 = (unsigned int)cin.channel[15] & 0x07FF;

    return _crsf_send_packet(
        pHndl->pSerial, CRSF_CHANNEL_BYTES, CRSFMsgRC, (void*) &msg);
}

/**
 * @brief Generic Internal read function
 *
 * @param pHndl CRSF Handle
 * @param src Source Data
 * @param dst Target Copy
 * @param size size of internal data
 * @return
 */
eCRSFError _crsf_read(CRSF_t* pHndl, void* src, void* dst, size_t size) {
    if (!pHndl)
        return eCRSFNULL;
    if (!src)
        return eCRSFNULL;
    if (!dst)
        return eCRSFNULL;
    if (pHndl->state != eCRSFOK)
        return pHndl->state;
    // Attempt to take the read semaphore from the buffering task
    if (xSemaphoreTake(pHndl->tx_hndl, CRSF_WAIT_TICKS) == pdTRUE) {
        memcpy(dst, src, size);
        xSemaphoreGive(pHndl->tx_hndl);
        return eCRSFOK;
    }
    return eCRSFSemFail;
}

eCRSFError crsf_read_gps(CRSF_t* pHndl, crsf_gps_t* pGPS) {
    return _crsf_read(pHndl, &pHndl->pkt.gps, pGPS, sizeof(crsf_gps_t));
}

eCRSFError crsf_read_battery(CRSF_t* pHndl, crsf_battery_t* pBattery) {
    return _crsf_read(pHndl, &pHndl->pkt.batt, pBattery, sizeof(crsf_battery_t));
}

eCRSFError crsf_read_attitude(CRSF_t* pHndl, crsf_attitude_t* pAttitude) {
    return _crsf_read(pHndl, &pHndl->pkt.att, pAttitude, sizeof(crsf_attitude_t));
}

eCRSFError crsf_read_mode(CRSF_t* pHndl, crsf_fcmode_t* pMode) {
    return _crsf_read(pHndl, &pHndl->pkt.mode, pMode, sizeof(crsf_fcmode_t));
}
