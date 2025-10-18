/**
 * @file csrf_types.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2025-10-14
 * @modified Last Modified: 2025-10-14
 *
 * @copyright Copyright (c) 2025
 */

#ifndef _CRSF_TYPES_H_
#define _CRSF_TYPES_H_

#include <stdint.h>

#define CRSF_DATA_MAXLEN 64
#define CRSF_N_CHANNELS 16
#define CRSF_CHANNEL_BYTES ((CRSF_N_CHANNELS * 11) / 8)
#define CRSF_STR_LEN 10

#define CRSF_CHANNEL_MIN 1000
#define CRSF_CHANNEL_ZERO 1500
#define CRSF_CHANNEL_MAX 2000

enum eCRSFDataType {
    UINT8 = 0,
    INT8 = 1,
    UINT16 = 2,
    INT16 = 3,
    FLOAT = 8,
    TEXT_SELECTION = 9,
    STRING = 10,
    FOLDER = 11,
    INFO = 12,
    COMMAND = 13,
    OUT_OF_RANGE = 127
};

enum eCRSFMsgId {
    CRSFMsgRC = 0x16,
    CRSFMsgLinkStat = 0x14,
    CRSFMsgBatt = 0x08,
    CRSFMsgFlightMode = 0x32,
};

typedef struct {
    float lattitude;
    float longitude;
    float groundspeed;
    float heading;
    // Altitude (m)
    float altitude;
    uint8_t sat_count;
} crsf_gps_t;

typedef struct {
    float voltage;
    // mA
    float current;
    // mAh
    uint32_t capacity;
    uint8_t percent_remaining;
} crsf_battery_t;

enum eCRSFAntMode {
    eCRSFAnt_1,
    eCRSFAnt_2
};

enum eCRSFTxPower {
    eCRSFTxPower_0mW,
    eCRSFTxPower_10mW,
    eCRSFTxPower_25mW,
    eCRSFTxPower_100mW,
    eCRSFTxPower_500mW,
    eCRSFTxPower_1000mW,
    eCRSFTxPower_2000mW,
};

typedef struct {
    // dBm *-1
    uint8_t uplink_RSSI_1;
    // dBm *-1
    uint8_t uplink_RSSI_2;
    // percent
    uint8_t uplink_quality;
    // uplink SNR (db)
    int8_t uplink_SNR;
    // enum ant 1 = 0, 2
    uint8_t diversity_active_antenna;
    // enum Mode (4fps = 0, 50fps, 150Hz)
    enum eCRSFAntMode RF_mode;
    // enum (0mW, 10mW, 25mW, 100mW, 500mW, 1000mW, 2000mW)
    enum eCRSFTxPower tx_power;
    // dBm * -1
    uint8_t downlink_RSSI;
    // percent
    uint8_t downlink_quality;
    // db
    int8_t downlink_SNR;
} crsf_link_t;

typedef struct {
    union {
        struct {
            uint16_t chan0;
            uint16_t chan1;
            uint16_t chan2;
            uint16_t chan3;
            uint16_t chan4;
            uint16_t chan5;
            uint16_t chan6;
            uint16_t chan7;
            uint16_t chan8;
            uint16_t chan9;
            uint16_t chan10;
            uint16_t chan11;
            uint16_t chan12;
            uint16_t chan13;
            uint16_t chan14;
            uint16_t chan15;
        };
        uint16_t channel[CRSF_N_CHANNELS];
    };
} crsf_rc_t;

typedef struct {
    float pitch;
    float roll;
    float yaw;
} crsf_attitude_t;

typedef struct {
    char mode[CRSF_STR_LEN];
} crsf_fcmode_t;

#endif
