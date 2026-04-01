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

#define RPLIDAR_STACK_SIZE (configMINIMAL_STACK_SIZE << 2)

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
} __attribute__((packed)) RpLidarRequest;

typedef struct {
    uint8_t start_flag;
    uint8_t command;
    uint8_t payload_size;
    uint8_t payload[255];
    uint8_t checksum;
} __attribute__((packed)) RpLidarRequestWithPayload;

#define START_FLAG 0xA5

typedef uint8_t eRpLidarCommand;
#define COMMAND_STOP                0x25
#define COMMAND_RESET               0x40
#define COMMAND_SCAN                0x20
#define COMMAND_EXPRESS_SCAN        0x82
#define COMMAND_GET_INFO            0x50
#define COMMAND_GET_HEALTH          0x52
#define COMMAND_GET_SAMPLERATE      0x59
#define COMMAND_GET_LIDAR_CONF      0x84
#define COMMAND_MOTOR_SPEED_CTRL    0xA8


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
#define RPLIDAR_CONF_SCAN_MODE_ANS_TYPE         0x75
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
    uint8_t check : 1; // Check bit, constantly set to 1. Can be used as a data check bit.
    uint16_t angle_q6 : 15; // The measurement heading angle related to RPLIDAR’s heading. In degree unit, [0-360) Stored using fix point number. Actual angle = angle_q6/64.0 Degree
    uint16_t distance_q2 : 16; // Measured object distance related to RPLIDAR’s rotation center. In millimeter (mm) unit. Represents using fix point. Set to 0 when the measurement is invalid. Actual Distance = distance_q2/4.0 mm
} __attribute__((packed)) RpLidarScanDataResponse;

typedef struct {
    uint8_t checksum1 : 4;
    uint8_t sync1 : 4;
    uint8_t checksum2 : 4;
    uint8_t sync2 : 4;
    uint16_t start_angle_q6 : 15;
    uint8_t start : 1;
} __attribute__((packed)) RpLidarExpressScanDataResponseHeader;

#define RPLIDAR_EXPRESS_SCAN_CABIN_SIZE 40U

typedef uint16_t RpLidarExpressScanDataResponseDistances[RPLIDAR_EXPRESS_SCAN_CABIN_SIZE];

typedef struct {
    RpLidarExpressScanDataResponseHeader header;
    RpLidarExpressScanDataResponseDistances cabin;
} __attribute__((packed)) RpLidarExpressScanDataResponse;

#define SYNC1 0xA
#define SYNC2 0x5

static inline void print_RpLidarRequestNoPayload(uint16_t indent, RpLidarRequest rnp) {
    printf("%*sstart_flag: 0x%02X\n", indent, "", rnp.start_flag);
    printf("%*scommand: 0x%02X\n", indent, "", rnp.command);
}

static inline void print_RpLidarRequestWithPayload(uint16_t indent, RpLidarRequestWithPayload *rwp) {
    printf("%*sstart_flag: 0x%02X\n", indent, "", rwp->start_flag);
    printf("%*scommand: 0x%02X\n", indent, "", rwp->command);
    printf("%*spayload_size: 0x%02X\n", indent, "", rwp->payload_size);
    printf("%*spayload:", indent, "");
    uint8_t i = 0, *payload = rwp->payload;
    do {
        printf(" %02X", payload[i]);
    } while (++i < rwp->payload_size);
    printf("\n");
    printf("%*schecksum: 0x%02X\n", indent, "", rwp->checksum);
}

static inline void print_RpLidarResponseDescriptor(uint16_t indent, RpLidarResponseDescriptor rd) {
    printf("%*sstart_flag1: 0x%02X\n", indent, "", rd.start_flag1);
    printf("%*sstart_flag2: 0x%02X\n", indent, "", rd.start_flag2);
    printf("%*sdata_response_length: %u\n", indent, "", rd.data_response_length);
    printf("%*ssend_mode: 0x%1X\n", indent, "", rd.send_mode); 
    printf("%*sdata_type: 0x%02X\n", indent, "", rd.data_type);  
}

static inline void print_RpLidarDeviceInfo(uint16_t indent, RpLidarDeviceInfo device_info) {
    printf("%*smajor_model: %X\n", indent, "", device_info.major_model);
    printf("%*ssub_model: %X\n", indent, "", device_info.sub_model);
    printf("%*sfirmware_minor: %X\n", indent, "", device_info.firmware_minor);
    printf("%*sfirmware_major: %X\n", indent, "", device_info.firmware_major);
    printf("%*shardware: %X\n", indent, "", device_info.hardware);
    printf("%*sserialnumber: %X-%X-%X-%X-%X-%X-%X-%X-%X-%X-%X-%X-%X-%X-%X-%X\n", 
         indent, "",
         device_info.serialnumber[0], 
         device_info.serialnumber[1], 
         device_info.serialnumber[2],
         device_info.serialnumber[3],
         device_info.serialnumber[4],
         device_info.serialnumber[5],
         device_info.serialnumber[6],
         device_info.serialnumber[7],
         device_info.serialnumber[8],
         device_info.serialnumber[9],
         device_info.serialnumber[10],
         device_info.serialnumber[11],
         device_info.serialnumber[12],
         device_info.serialnumber[13],
         device_info.serialnumber[14],
         device_info.serialnumber[15]
        );
}

static inline void print_RpLidarDeviceHealth(uint16_t indent, RpLidarDeviceHealth device_health) {
    printf("%*sstatus: %u\n", indent, "", device_health.status);
    printf("%*serror_code: %u\n", indent, "", device_health.error_code);
}

static inline void print_RpLidarSampleRate(uint16_t indent, RpLidarSampleRate sample_rate) {
    printf("%*sTstandard: %u\n", indent, "", sample_rate.Tstandard);
    printf("%*sTexpress: %u\n", indent, "", sample_rate.Texpress);
}

static inline void print_RpLidarConfResponseDataWithU8Payload(const uint16_t indent, const RpLidarConfResponseDataWithU8Payload data, const char *const payload_name) {
    printf("%*stype: 0x%02lX\n", indent, "", data.type);
    printf("%*s%s: 0x%02X\n", indent, "", payload_name, data.payload);
}

static inline void print_RpLidarConfResponseDataWithU16Payload(const uint16_t indent, const RpLidarConfResponseDataWithU16Payload data, const char *const payload_name) {
    printf("%*stype: 0x%02lX\n", indent, "", data.type);
    printf("%*s%s: %u\n", indent, "", payload_name, data.payload);
}

static inline void print_RpLidarConfResponseDataWithU32Payload(const uint16_t indent, const RpLidarConfResponseDataWithU32Payload data, const char *const payload_name) {
    printf("%*stype: 0x%02lX\n", indent, "", data.type);
    printf("%*s%s: %lu\n", indent, "", payload_name, data.payload);
}

static inline void print_RpLidarConfResponseDataWithStringPayload(const uint16_t indent, const RpLidarConfResponseDataWithStringPayload data, const char *const payload_name) {
    printf("%*stype: 0x%02lX\n", indent, "", data.type);
    printf("%*s%s: %s\n", indent, "", payload_name, data.payload);
}

static inline void print_RpLidarScanDataResponse(const uint16_t indent, const RpLidarScanDataResponse scan) {
    printf("%*sstart: %u\n", indent, "", scan.start);
    printf("%*sn_start: %u\n", indent, "", scan.n_start);
    printf("%*squality: %u\n", indent, "", scan.quality);
    printf("%*scheck: %u\n", indent, "", scan.check);
    printf("%*sangle_q6: %d\n", indent, "", scan.angle_q6);
    printf("%*sdistance_q2: %d\n", indent, "", scan.distance_q2);
}

static inline void print_RpLidarExpressScanDataResponseHeader(uint16_t const indent, RpLidarExpressScanDataResponseHeader const*header) {
    printf("%*schecksum1: %01X\n", indent, "", header->checksum1);
    printf("%*ssync1: %01X\n", indent, "", header->sync1);
    printf("%*schecksum2: %01X\n", indent, "", header->checksum2);
    printf("%*ssync2: %01X\n", indent, "", header->sync2);
    printf("%*sstart_angle_q6: %u\n", indent, "", header->start_angle_q6);
    printf("%*sstart: %u\n", indent, "", header->start);
}

static inline void print_RpLidarExpressScanDataResponse(const uint16_t indent, const RpLidarExpressScanDataResponse *scan) {
    print_RpLidarExpressScanDataResponseHeader(indent, &scan->header);
    printf("%*scabin: %u\n", indent, "", scan->cabin[0]);
    for (uint8_t i = 1; i < sizeof(scan->cabin); ++i)
        printf("%*s       %u\n", indent, "", scan->cabin[i]);
}

typedef QueueHandle_t RpLidarQueueHandle_t;

typedef struct {
    // enum eCBLidar id; eCBLidarFront = 0U; eCBLidarVertical = 1U;
    uint8_t id;
    Serial_t* pSerial;

    // Task information (Maybe not needed)
    TaskHandle_t tsk_hndl;
    StaticTask_t tsk_buf;
    StackType_t tsk_stack[RPLIDAR_STACK_SIZE];

    // Recieve Buffer (from serial driver interrupt)
    StreamBufferHandle_t rx_hndl;
    StaticStreamBuffer_t rx_streamBuf;
    uint8_t rx_buf[RPLIDAR_BUF_LEN];

    // Output Queue
    QueueHandle_t tx;

    eRpLidarError state;
} RpLidar_t;

/**
 * @brief Initalize an RPLiDAR device
 *
 * @param pHndl Device Handle
 * @param id id number from enum CBLidar
 * @param pSerial Serial Connection the device is on
 * @param tx output Queue
//  * @param stx Serial TX pin
//  * @param srx Serial RX pin
 * @return
 */
extern eRpLidarError rplidar_init(RpLidar_t* pHndl,
                                  uint8_t id,
                                  Serial_t* pSerial,
                                  QueueHandle_t output/*,
                                  pin_t stx,
                                  pin_t srx*/);
#endif
