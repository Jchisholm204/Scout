/**
 * @file rplidar.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2025-10-17
 * @modified Last Modified: 2025-10-17
 *
 * @copyright Copyright (c) 2025
 */

#ifndef _RPLIDAR_H_
#define _RPLIDAR_H_
#include "drivers/serial.h"

// from https://download-en.slamtec.com/api/download/rplidar-c1-datasheet/1?lang=en at page 14
#define RPLIDAR_BAUD 460800

#define RPLIDAR_N_POINTS 360

#define RPLIDAR_NOTIFY_NEW 0x1
#define RPLIDAR_NOTIFY_OLD 0x2

#define RPLIDAR_BUF_LEN 1000

typedef enum {
    eRpLidarOK,
    eRpLidarNULL,
    eRpLidarNoInit,
    eRpLidarInitFail,
    eRpLidarTskCreateFail,
    eRpLidarSerialFail,
    eRpLidarStreamFail,
    eRpLidarSemFail,
    eRpLidarNoPkt,
} eRpLidarError;

typedef struct {
    uint8_t start_flag;
    uint8_t command;
} __attribute__((packed)) RpLidarRequestNoPayload;

typedef struct {
    uint8_t start_flag;
    uint8_t command;
    uint8_t payload_size;
    uint8_t payload[256];
    uint8_t checksum;
} __attribute__((packed)) RpLidarRequestWithPayload;

#define START_FLAG 0xA5

typedef uint8_t eRpLidarCommand;
#define COMMAND_STOP            0x25
#define COMMAND_RESET           0x40
#define COMMAND_SCAN            0x20
#define COMMAND_EXPRESS_SCAN    0x82
#define COMMAND_GET_INFO        0x50
#define COMMAND_GET_HEALTH      0x52
#define COMMAND_GET_SAMPLERATE  0x59
#define COMMAND_GET_LIDAR_CONF  0x84


typedef struct {
    uint32_t type;
} __attribute__((packed)) RpLidarConfRequestDataNoPayload;

typedef struct {
    uint32_t type;
    uint16_t payload;
} __attribute__((packed)) RpLidarConfRequestDataWithU16Payload;


typedef uint8_t eRpLidarConfigEntryType;
#define RPLIDAR_CONF_SCAN_MODE_COUNT            0x70
#define RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE    0x71
#define RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE     0x74
#define RPLIDAR_CONF_SCAN_MODE_ANS_TYPE         0x74
#define RPLIDAR_CONF_SCAN_MODE_TYPICAL          0x7C
#define RPLIDAR_CONF_SCAN_MODE_NAME             0x7F

typedef struct {
    uint32_t type;
    uint8_t payload;
} __attribute__((packed)) RpLidarConfResponseDataWithU8Payload;

typedef struct {
    uint32_t type;
    uint16_t payload;
} __attribute__((packed)) RpLidarConfResponseDataWithU16Payload;

typedef struct {
    uint32_t type;
    uint32_t payload;
} __attribute__((packed)) RpLidarConfResponseDataWithU32Payload;

typedef struct {
    uint32_t type;
    char payload[64]; // '\0' terminated string, so this may need to be longer.
} __attribute__((packed)) RpLidarConfResponseDataWithStringPayload;

typedef struct {
    uint8_t start_flag1; // 0xA5
    uint8_t start_flag2; // 0x5A
    uint32_t data_response_length : 30; // 30bits for data length in bytes
    uint8_t send_mode : 2; // 2bits for send mode: 00 for single request - single response, 01 for single request - multiple response
    uint8_t data_type; 
} __attribute__((packed)) RpLidarResponseDescriptor;

#define START_FLAG1 0xA5
#define START_FLAG2 0x5A

typedef struct {
    uint8_t major_model : 4;
    uint8_t sub_model : 4;
    uint8_t firmware_minor;
    uint8_t firmware_major;
    uint8_t hardware;
    uint8_t serialnumber[16];
} __attribute__((packed)) RpLidarDeviceInfo;

typedef struct {
    uint8_t status; // 0: Good 1: Warning 2: Error
    uint16_t error_code;
} __attribute__((packed)) RpLidarDeviceHealth;

typedef struct {
    uint16_t Tstandard; // unit: microsecond
    uint16_t Texpress; // unit: microsecond
} __attribute__((packed)) RpLidarSampleRate;

typedef struct {
    uint8_t start : 1; // Start flag bit of a new scan. When start is set to 1, the current and incoming packets belong to a new 360 degree scan.
    uint8_t n_start : 1; // Inversed start flag bit, always has n_start = !start. Can be used as a data check bit.
    uint8_t quality : 6;
    uint16_t check : 1; // Check bit, constantly set to 1. Can be used as a data check bit.
    uint16_t angle_q6 : 15; // The measurement heading angle related to RPLIDAR’s heading. In degree unit, [0-360) Stored using fix point number. Actual angle = angle_q6/64.0 Degree
    uint16_t distance_q2 : 16; // Measured object distance related to RPLIDAR’s rotation center. In millimeter (mm) unit. Represents using fix point. Set to 0 when the measurement is invalid. Actual Distance = distance_q2/4.0 mm
} __attribute__((packed)) RpLidarScan;

typedef struct {
    float angle;
    float distance;
} RpLidarPoint_t;

typedef RpLidarPoint_t RpLidarScanArray[RPLIDAR_N_POINTS];

typedef struct {
    Serial_t* pSerial;

    // Task information (Maybe not needed)
    TaskHandle_t tsk_hndl;
    StaticTask_t tsk_buf;
    StackType_t tsk_stack[configMINIMAL_STACK_SIZE];

    // Recieve Buffer (from serial driver interrupt)
    StreamBufferHandle_t rx_hndl;
    StaticStreamBuffer_t rx_streamBuf;
    uint8_t rx_buf[RPLIDAR_BUF_LEN];

    // Internal
    RpLidarScanArray scan;

    eRpLidarError state;
} RpLidar_t;

/**
 * @brief Initalize an RPLiDAR device
 *
 * @param pHndl Device Handle
 * @param pSerial Serial Connection the device is on
 * @param stx Serial TX pin
 * @param srx Serial RX pin
 * @return
 */
extern eRpLidarError rplidar_init(RpLidar_t* pHndl,
                                  Serial_t* pSerial,
                                  pin_t stx,
                                  pin_t srx);

/**
 * @brief Attach a notifier to the LiDAR device
 *
 * @param pHndl LiDAR device handle
 * @param pTask Task to notify
 * @return
 */
extern eRpLidarError rplidar_notify(RpLidar_t* pHndl,
                                    TaskHandle_t* const pNotify_tskhndl);

/**
 * @brief Read the latest scan from the LiDAR
 *
 * @param pHndl Device handle to read from
 * @param pScan Address to place scan data in
 * @return
 */
extern eRpLidarError rplidar_read(RpLidar_t* pHndl, RpLidarScanArray* pScan);

#endif
