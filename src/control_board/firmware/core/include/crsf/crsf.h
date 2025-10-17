/**
 * @file crsf.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2025-10-14
 * @modified Last Modified: 2025-10-14
 *
 * @copyright Copyright (c) 2025
 */

#ifndef _CSRF_H_
#define _CSRF_H_
#include "crsf_types.h"
#include "drivers/serial.h"
#include "hal/pin.h"

#define CRSF_BAUD 420000
#define CRSF_SERIAL_LOCK 0x1234
#define CRSF_WAIT_TICKS 10

typedef enum {
    eCSRFOK,
    eCRSFNULL,
    eCRSFNoInit,
    eCRSFInitFail,
    eCRSFTskCreateFail,
    eCRSFSerialFail,
    eCRSFSemFail,
    eCRSFNoPkt,
} eCRSFError;

typedef struct CRSF {
    Serial_t* pSerial;

    // Task information (Maybe not needed)
    TaskHandle_t tsk_hndl;
    StaticTask_t tsk_buf;
    StackType_t tsk_stack[configMINIMAL_STACK_SIZE];

    // Recieve Buffer (from serial driver interrupt)
    StreamBufferHandle_t rx_hndl;
    StaticStreamBuffer_t rx_streamBuf;
    uint8_t rx_buf[configMINIMAL_STACK_SIZE];

    SemaphoreHandle_t tx_hndl;
    StaticSemaphore_t static_tx_semphr;

    // CRSF Packets
    struct crsf_packets{
        crsf_gps_t gps;
        crsf_battery_t batt;
        crsf_rc_t rc;
        crsf_attitude_t att;
        crsf_fcmode_t mode;
    } pkt;
    eCRSFError state;
} CRSF_t;

extern eCRSFError crsf_init(CRSF_t* pHndl, Serial_t* pSerial, pin_t srx,
                            pin_t stx);
extern eCRSFError crsf_write_rc(CRSF_t* pHndl, crsf_rc_t *pChannels);
extern eCRSFError crsf_read_gps(CRSF_t* pHndl, crsf_gps_t *pGPS);
extern eCRSFError crsf_read_battery(CRSF_t* pHndl, crsf_battery_t *pBattery);
extern eCRSFError crsf_read_attitude(CRSF_t* pHndl, crsf_attitude_t *pAttitude);
extern eCRSFError crsf_read_mode(CRSF_t* pHndl, crsf_fcmode_t *pMode);

#endif
