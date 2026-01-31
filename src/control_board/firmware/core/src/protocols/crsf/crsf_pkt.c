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
#include "memory.h"
#include "protocols/crsf/crsf.h"
#include "protocols/crsf/crsf_types.h"
#include "protocols/crsf/crsf_types_internal.h"

extern uint8_t _crsf_crc8(const uint8_t *ptr, uint8_t len);

eCRSFError _crsf_send_packet(CRSF_t *pHndl,
                             _crsf_msg_t *msg,
                             enum eCRSFMsgId id,
                             uint8_t len) {
    if (!pHndl)
        return eCRSFNULL;
    if (!msg)
        return eCRSFNULL;

    if (len >= CRSF_DATA_MAXLEN) {
        return eCRSFPktOverLen;
    }

    // Addr = CRSF Addr FC
    msg->addr = CRSF_ADDR;
    msg->length = len + 2; // type + payload + crc
    msg->type = (uint8_t) id;

    // CRC includes type and payload
    uint8_t crc = _crsf_crc8(&msg->type, len + 1);
    msg->pyld[len] = crc;

    xStreamBufferSend(pHndl->tx.pBuf_hndl, msg, msg->length + 2, 10);

    return eCRSFOK;
}

eCRSFError crsf_write_rc(CRSF_t *pHndl, crsf_rc_t *pChannels) {
    if (!pHndl || !pChannels) {
        return eCRSFNULL;
    }
    _crsf_msg_t msg;
    msg.rc.chan0 = pChannels->chan0;
    msg.rc.chan1 = pChannels->chan1;
    msg.rc.chan2 = pChannels->chan2;
    msg.rc.chan3 = pChannels->chan3;
    msg.rc.chan4 = pChannels->chan4;
    msg.rc.chan5 = pChannels->chan5;
    msg.rc.chan6 = pChannels->chan6;
    msg.rc.chan7 = pChannels->chan7;
    msg.rc.chan8 = pChannels->chan8;
    msg.rc.chan9 = pChannels->chan9;
    msg.rc.chan10 = pChannels->chan10;
    msg.rc.chan11 = pChannels->chan11;
    msg.rc.chan12 = pChannels->chan12;
    msg.rc.chan13 = pChannels->chan13;
    msg.rc.chan14 = pChannels->chan14;
    msg.rc.chan15 = pChannels->chan15;
    return _crsf_send_packet(pHndl, &msg, CRSFMsgRC, sizeof(_crsf_rc_t));
}

eCRSFError crsf_write_battery(CRSF_t *pHndl, crsf_battery_t *pBattery) {
}
eCRSFError crsf_write_attitude(CRSF_t *pHndl, crsf_attitude_t *pAttitude) {
}
eCRSFError crsf_write_mode(CRSF_t *pHndl, crsf_fcmode_t *pMode) {
}

eCRSFError crsf_read_rc(CRSF_t *pHndl, crsf_rc_t *pChannels) {
    if (!pHndl || !pChannels) {
        return eCRSFNULL;
    }
    memcpy(pChannels, &pHndl->pkt.rc, sizeof(crsf_rc_t));
    return eCRSFOK;
}

eCRSFError crsf_read_battery(CRSF_t *pHndl, crsf_battery_t *pBattery) {
    if (!pHndl || !pBattery) {
        return eCRSFNULL;
    }
    memcpy(pBattery, &pHndl->pkt.batt, sizeof(crsf_battery_t));
    return eCRSFOK;
}

eCRSFError crsf_read_attitude(CRSF_t *pHndl, crsf_attitude_t *pAttitude) {
    if (!pHndl || !pAttitude) {
        return eCRSFNULL;
    }
    memcpy(pAttitude, &pHndl->pkt.att, sizeof(crsf_attitude_t));
    return eCRSFOK;
}

eCRSFError crsf_read_mode(CRSF_t *pHndl, crsf_fcmode_t *pMode) {
    if (!pHndl || !pMode) {
        return eCRSFNULL;
    }
    memcpy(pMode, &pHndl->pkt.mode, sizeof(crsf_fcmode_t));
    return eCRSFOK;
}
