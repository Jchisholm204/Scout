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
    eRpLidarSemFail,
    eRpLidarNoPkt,
} eRpLidarError;

typedef struct {
    uint8_t start_flag1; // 0xA5
    uint8_t start_flag2; // 0x5A
    uint32_t data_response_detail; // 30bits for data length in bytes, 2bits for send mode: 00 for single response, 01 for multiple response
    uint8_t data_type; 
} RpLidarResponseDescriptor __attribute__((packed));

#define RPLIDAR_RESPONSE_DESCRIPTOR ((RpLidarResponseDescriptor){.start_flag1=0xA5, .start_flag2=0x5A, .data_response_detail=0, .data_type=0})

typedef struct {
    float angle;
    float distance;
} RpLidarPoint_t;

typedef RpLidarPoint_t RpLidarScan[RPLIDAR_N_POINTS];

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
    RpLidarScan scan;

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
extern eRpLidarError rplidar_read(RpLidar_t* pHndl, RpLidarScan* pScan);

#endif
