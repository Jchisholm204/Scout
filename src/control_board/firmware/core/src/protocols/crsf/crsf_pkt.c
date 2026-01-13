/**
 * @file crsf_pkt.c
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2025-10-18
 * @modified Last Modified: 2025-10-18
 *
 * @copyright Copyright (c) 2025
 */

#define CRSF_INTERNAL
#include "protocols/crsf/crsf.h"
#include "protocols/crsf/crsf_types.h"
#include "protocols/crsf/crsf_types_internal.h"
#include "memory.h"

extern uint8_t _crsf_crc8(const uint8_t* ptr, uint8_t len);

eCRSFError _crsf_send_packet(CRSF_t *pHndl, _crsf_msg_t *msg, enum eCRSFMsgId id, uint8_t len) {
    if (!pHndl)
        return eCRSFNULL;
    if (!msg)
        return eCRSFNULL;

    if(len >= CRSF_DATA_MAXLEN){
        return eCRSFPktOverLen;
    }

    // Addr = CRSF Addr FC
    msg->addr = CRSF_ADDR;
    msg->length = len + 2; // type + payload + crc
    msg->type = (uint8_t) id;

    // CRC includes type and payload
    uint8_t crc = _crsf_crc8(&msg->type, len + 1);
    msg->pyld[len] = crc;

    if(xSemaphoreTake(pHndl->tx.semphr_hndl, 10) != pdTRUE){
        xStreamBufferSend(*pHndl->tx.pBuf_hndl, msg, msg->length + 1, 10);
    }

    return eCRSFOK;
}


eCRSFError crsf_read_rc(CRSF_t *pHndl, crsf_rc_t *pChannels){
    if(!pHndl || !pChannels){
        return eCRSFNULL;
    }
    memcpy(pChannels, &pHndl->pkt.rc, sizeof(crsf_rc_t));
    return eCRSFOK;
}

eCRSFError crsf_read_battery(CRSF_t *pHndl, crsf_battery_t *pBattery){
    if(!pHndl || !pBattery){
        return eCRSFNULL;
    }
    memcpy(pBattery, &pHndl->pkt.batt, sizeof(crsf_battery_t));
    return eCRSFOK;
}

eCRSFError crsf_read_attitude(CRSF_t *pHndl, crsf_attitude_t *pAttitude){
    if(!pHndl || !pAttitude){
        return eCRSFNULL;
    }
    memcpy(pAttitude, &pHndl->pkt.att, sizeof(crsf_attitude_t));
    return eCRSFOK;
}

eCRSFError crsf_read_mode(CRSF_t *pHndl, crsf_fcmode_t *pMode){
    if(!pHndl || !pMode){
        return eCRSFNULL;
    }
    memcpy(pMode, &pHndl->pkt.mode, sizeof(crsf_fcmode_t));
    return eCRSFOK;
}

