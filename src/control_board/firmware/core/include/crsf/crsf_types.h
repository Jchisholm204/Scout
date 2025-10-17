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

#define CRSF_DATA_MAXLEN 4
#define CRSF_N_CHANNELS 16
#define CRSF_CHANNEL_BYTES ((CRSF_N_CHANNELS*11)/8)
#define CRSF_STR_LEN 10

#define CRSF_CHANNEL_MIN 1000
#define CRSF_CHANNEL_ZERO 1500
#define CRSF_CHANNEL_MAX 2000

typedef struct {
    uint8_t addr;
    uint8_t length;
    uint8_t type;
    uint8_t data[CRSF_DATA_MAXLEN];
    uint8_t crc;
} crsf_msg_t;

typedef struct __attribute__((packed)) {
    // degrees / 10_000_000
    int32_t lattitude;
    // degrees / 10_000_000
    int32_t longitude;
    // km/h / 100
    uint16_t groundspeed;
    // degree / 100
    uint16_t heading;
    // meter - 1000m offset
    uint16_t altitude;
    uint8_t sat_count;
} crsf_gps_t;

typedef struct __attribute__((packed)) {
    // mV * 100
    uint16_t voltage;
    // mA * 100
    uint16_t current;
    // mAh (24 bits)
    uint8_t capacity[3];
    // percent (0-100]
    uint8_t percent_remaining;
} crsf_battery_t;

typedef struct __attribute__((packed)) {
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
    uint8_t RF_mode;
    // enum (0mW, 10mW, 25mW, 100mW, 500mW, 1000mW, 2000mW)
    uint8_t tx_power;
    // dBm * -1
    uint8_t downlink_RSSI;
    // percent
    uint8_t downlink_quality;
    // db
    int8_t downlink_SNR;
} crsf_link_t;

typedef struct __attribute__((packed)) {
    union __attribute__((packed)) {
        uint8_t raw[CRSF_CHANNEL_BYTES];
        uint16_t channel[CRSF_N_CHANNELS];
    };
} crsf_rc_t;

typedef struct {
    int16_t pitch;
    int16_t roll;
    int16_t yaw;
} crsf_attitude_t;

typedef struct {
    char mode[CRSF_STR_LEN];
} crsf_fcmode_t;

#endif
