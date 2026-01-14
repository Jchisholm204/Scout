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

#include "protocols/crsf/crsf.h"

#include "memory.h"
#include "pin_cfg.h"
#include "protocols/crsf/crsf_types.h"
#include "protocols/crsf/crsf_types_internal.h"

void vCRSF_Hndl_tsk(void *pvParams);
uint8_t _crsf_crc8(const uint8_t *ptr, uint8_t len);

StreamBufferHandle_t crsf_init(CRSF_t *pHndl, StreamBufferHandle_t pTx_hndl) {
    if (!pHndl)
        return NULL;
    if (!pTx_hndl)
        return NULL;

    // Setup Write handle
    pHndl->tx.semphr_hndl =
        xSemaphoreCreateMutexStatic(&pHndl->tx.static_semphr);
    pHndl->tx.pBuf_hndl = pTx_hndl;

    // Setup Read Handle
    pHndl->rx.hndl = xStreamBufferCreateStatic(
        configMINIMAL_STACK_SIZE, 1, pHndl->rx.buf, &pHndl->rx.static_stream);

    // Setup the CRSF rx task
    pHndl->tsk.hndl = xTaskCreateStatic(vCRSF_Hndl_tsk,
                                        "CRSF",
                                        CRSF_STACK_SIZE,
                                        (void *) pHndl,
                                        2,
                                        pHndl->tsk.stack,
                                        &pHndl->tsk.static_tsk);
    if (!pHndl->tsk.hndl) {
        pHndl->state = eCRSFTskCreateFail;
        return NULL;
    }

    return pHndl->rx.hndl;
}

void vCRSF_Hndl_tsk(void *pvParams) {
    if (!pvParams)
        return;
    CRSF_t *pHndl = (CRSF_t *) pvParams;
    if (!pHndl) {
        return;
    }
    uint8_t rx_buf[CRSF_DATA_MAXLEN] = {0};
    uint8_t rx_idx = 0;

    if (pHndl->state != eCRSFOK) {
        return;
    }

    for (;;) {
        uint8_t new_byte = 0x00;
        // Wait for timeout duration to receive bytes from the buffer
        size_t n_rx =
            xStreamBufferReceive(pHndl->rx.hndl, &new_byte, 1, CRSF_TIMEOUT_MS);

        // Timeout Checks
        if (n_rx == 0) {
            pHndl->state = eCRSFTimeout;
            // printf("RX0\n");
            continue;
        }

        rx_buf[rx_idx++] = new_byte;

        // First byte failed to be the ID byte
        if (rx_idx == 1 && new_byte != CRSF_ADDR) {
            rx_idx = 0;
            pHndl->state = eCRSFAddrMisMatch;
            continue;
        }
        // Check that the length is not invalid
        if (rx_idx == 2 && (new_byte < 2 || new_byte > CRSF_DATA_MAXLEN)) {
            rx_idx = 0;
            pHndl->state = eCRSFPktOverLen;
            continue;
        }

        // Attempt to parse a packet
        if (rx_idx >= 2 && rx_idx == rx_buf[1] + 2) {
            _crsf_msg_t *msg = (_crsf_msg_t *) rx_buf;
            // Check if the incoming packet has a valid CRC
            uint8_t crc_check =
                _crsf_crc8((void *) &msg->type, msg->length - 1);
            uint8_t crc_msg = ((uint8_t *) msg)[msg->length + 1];
            // Unpack the message if the crc matches
            if (crc_msg == crc_check) {
                switch ((enum eCRSFMsgId) msg->type) {
                case CRSFMsgRC:
                    pHndl->pkt.rc.chan0 = msg->rc.chan0;
                    pHndl->pkt.rc.chan1 = msg->rc.chan1;
                    pHndl->pkt.rc.chan2 = msg->rc.chan2;
                    pHndl->pkt.rc.chan3 = msg->rc.chan3;
                    pHndl->pkt.rc.chan4 = msg->rc.chan4;
                    pHndl->pkt.rc.chan5 = msg->rc.chan5;
                    pHndl->pkt.rc.chan6 = msg->rc.chan6;
                    pHndl->pkt.rc.chan7 = msg->rc.chan7;
                    pHndl->pkt.rc.chan8 = msg->rc.chan8;
                    pHndl->pkt.rc.chan9 = msg->rc.chan9;
                    pHndl->pkt.rc.chan10 = msg->rc.chan10;
                    pHndl->pkt.rc.chan11 = msg->rc.chan11;
                    pHndl->pkt.rc.chan12 = msg->rc.chan12;
                    pHndl->pkt.rc.chan13 = msg->rc.chan13;
                    pHndl->pkt.rc.chan14 = msg->rc.chan14;
                    pHndl->pkt.rc.chan15 = msg->rc.chan15;
                    break;
                case CRSFMsgLinkStat:
                    pHndl->pkt.link.uplink_RSSI_1 = msg->link.uplink_RSSI_1;
                    pHndl->pkt.link.uplink_RSSI_2 = msg->link.uplink_RSSI_2;
                    pHndl->pkt.link.uplink_quality = msg->link.uplink_quality;
                    pHndl->pkt.link.uplink_SNR = msg->link.uplink_SNR;
                    pHndl->pkt.link.diversity_active_antenna =
                        msg->link.diversity_active_antenna;
                    pHndl->pkt.link.RF_mode = msg->link.RF_mode;
                    pHndl->pkt.link.tx_power = msg->link.tx_power;
                    pHndl->pkt.link.downlink_RSSI = msg->link.downlink_RSSI;
                    pHndl->pkt.link.downlink_quality =
                        msg->link.downlink_quality;
                    pHndl->pkt.link.downlink_SNR = msg->link.downlink_SNR;
                    break;
                case CRSFMsgBatt:
                    pHndl->pkt.batt.voltage = (float) msg->batt.voltage / 10;
                    pHndl->pkt.batt.current = (float) msg->batt.current / 10;
                    pHndl->pkt.batt.capacity =
                        ((uint32_t) msg->batt.capacity[2] << 16) |
                        ((uint32_t) msg->batt.capacity[1] << 8) |
                        ((uint32_t) msg->batt.capacity[0]);
                    pHndl->pkt.batt.percent_remaining =
                        msg->batt.percent_remaining;
                    break;
                case CRSFMsgFlightMode:
                    memcpy(pHndl->pkt.mode.mode, msg->mode.mode, CRSF_STR_LEN);
                    break;
                case CRSFMsgAtt:
                    pHndl->pkt.att.pitch = (float) msg->att.pitch;
                    pHndl->pkt.att.yaw = (float) msg->att.yaw;
                    pHndl->pkt.att.roll = (float) msg->att.roll;
                    break;
                default:
                    break;
                }
            }
            // Reset the receive index buffer
            rx_idx = 0;
            pHndl->state = eCRSFOK;
        }
    }
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

uint8_t _crsf_crc8(const uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc = crc8tab[crc ^ *ptr++];
    }
    return crc;
}
