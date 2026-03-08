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

#define LIMIT(min, val, max)                                                   \
    ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))

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

eCRSFError crsf_write_rc(CRSF_t *pHndl, const crsf_rc_t *pChannels) {
    if (!pHndl || !pChannels) {
        return eCRSFNULL;
    }
    _crsf_msg_t msg;
    msg.rc.chan0 = LIMIT(CRSF_CHANNEL_MIN, pChannels->chan0, CRSF_CHANNEL_MAX);
    msg.rc.chan1 = LIMIT(CRSF_CHANNEL_MIN, pChannels->chan1, CRSF_CHANNEL_MAX);
    msg.rc.chan2 = LIMIT(CRSF_CHANNEL_MIN, pChannels->chan2, CRSF_CHANNEL_MAX);
    msg.rc.chan3 = LIMIT(CRSF_CHANNEL_MIN, pChannels->chan3, CRSF_CHANNEL_MAX);
    msg.rc.chan4 = LIMIT(CRSF_CHANNEL_MIN, pChannels->chan4, CRSF_CHANNEL_MAX);
    msg.rc.chan5 = LIMIT(CRSF_CHANNEL_MIN, pChannels->chan5, CRSF_CHANNEL_MAX);
    msg.rc.chan6 = LIMIT(CRSF_CHANNEL_MIN, pChannels->chan6, CRSF_CHANNEL_MAX);
    msg.rc.chan7 = LIMIT(CRSF_CHANNEL_MIN, pChannels->chan7, CRSF_CHANNEL_MAX);
    msg.rc.chan8 = LIMIT(CRSF_CHANNEL_MIN, pChannels->chan8, CRSF_CHANNEL_MAX);
    msg.rc.chan9 = LIMIT(CRSF_CHANNEL_MIN, pChannels->chan9, CRSF_CHANNEL_MAX);
    msg.rc.chan10 =
        LIMIT(CRSF_CHANNEL_MIN, pChannels->chan10, CRSF_CHANNEL_MAX);
    msg.rc.chan11 =
        LIMIT(CRSF_CHANNEL_MIN, pChannels->chan11, CRSF_CHANNEL_MAX);
    msg.rc.chan12 =
        LIMIT(CRSF_CHANNEL_MIN, pChannels->chan12, CRSF_CHANNEL_MAX);
    msg.rc.chan13 =
        LIMIT(CRSF_CHANNEL_MIN, pChannels->chan13, CRSF_CHANNEL_MAX);
    msg.rc.chan14 =
        LIMIT(CRSF_CHANNEL_MIN, pChannels->chan14, CRSF_CHANNEL_MAX);
    msg.rc.chan15 =
        LIMIT(CRSF_CHANNEL_MIN, pChannels->chan15, CRSF_CHANNEL_MAX);
    return _crsf_send_packet(pHndl, &msg, CRSFMsgRC, sizeof(_crsf_rc_t));
}

eCRSFError crsf_write_battery(CRSF_t *pHndl, const crsf_battery_t *pBattery) {
    if (!pHndl || !pBattery) {
        return eCRSFNULL;
    }
    _crsf_msg_t msg;
    msg.batt.voltage = (uint16_t) (LIMIT(0, pBattery->voltage, 100) * 10);
    msg.batt.current = (uint16_t) (LIMIT(0, pBattery->current, 500) * 10);
    uint32_t capacity = pBattery->capacity;
    msg.batt.capacity[0] = capacity & 0xFF;
    msg.batt.capacity[1] = (capacity >> 8) & 0xFF;
    msg.batt.capacity[2] = (capacity >> 16) & 0xFF;
    msg.batt.percent_remaining = LIMIT(0, pBattery->percent_remaining, 100);
    return _crsf_send_packet(pHndl, &msg, CRSFMsgBatt, sizeof(_crsf_battery_t));
}

eCRSFError crsf_write_attitude(CRSF_t *pHndl,
                               const crsf_attitude_t *pAttitude) {
    if (!pHndl || !pAttitude) {
        return eCRSFNULL;
    }
    _crsf_msg_t msg;
    msg.att.pitch =
        (int16_t) (LIMIT(INT16_MIN, pAttitude->pitch * 10000, INT16_MAX));
    msg.att.yaw =
        (int16_t) (LIMIT(INT16_MIN, pAttitude->yaw * 10000, INT16_MAX));
    msg.att.roll =
        (int16_t) (LIMIT(INT16_MIN, pAttitude->roll * 10000, INT16_MAX));
    return _crsf_send_packet(pHndl, &msg, CRSFMsgAtt, sizeof(_crsf_attitude_t));
}

eCRSFError crsf_write_mode(CRSF_t *pHndl, const crsf_fcmode_t *pMode) {
    if (!pHndl || !pMode) {
        return eCRSFNULL;
    }
    _crsf_msg_t msg;
    memcpy(msg.mode.mode, pMode->mode, CRSF_STR_LEN);
    return _crsf_send_packet(pHndl, &msg, CRSFMsgFlightMode, sizeof(_crsf_fcmode_t));
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
