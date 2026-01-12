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
