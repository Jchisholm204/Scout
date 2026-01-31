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
#include "FreeRTOS.h"
#include "crsf_types.h"
#include "semphr.h"
#include "stream_buffer.h"

#include <stdio.h>

#define CRSF_BAUD 420000
#define CRSF_TIMEOUT_MS 100
#define CRSF_STACK_SIZE (configMINIMAL_STACK_SIZE)

typedef enum {
    eCRSFOK,
    eCRSFNULL,
    eCRSFNoInit,
    eCRSFInitFail,
    eCRSFTskCreateFail,
    eCRSFSemFail,
    eCRSFNoPkt,
    eCRSFAddrMisMatch,
    eCRSFPktOverLen,
    eCRSFCRCErr,
    eCRSFIdNoMatch,
    eCRSFTimeout
} eCRSFError;

typedef struct CRSF {
    // Task information (Maybe not needed)
    struct {
        StackType_t stack[CRSF_STACK_SIZE];
        TaskHandle_t hndl;
        StaticTask_t static_tsk;
    } tsk;

    // Recieve Buffer (from serial driver interrupt)
    struct {
        StreamBufferHandle_t hndl;
        StaticStreamBuffer_t static_stream;
        uint8_t buf[configMINIMAL_STACK_SIZE];
    } rx;

    struct {
        SemaphoreHandle_t semphr_hndl;
        StaticSemaphore_t static_semphr;
        StreamBufferHandle_t pBuf_hndl;
    } tx;

    // CRSF Packets
    struct crsf_packets {
        crsf_link_t link;
        crsf_gps_t gps;
        crsf_battery_t batt;
        crsf_rc_t rc;
        crsf_attitude_t att;
        crsf_fcmode_t mode;
    } pkt;
    eCRSFError state;
} CRSF_t;

/**
 * @brief Initialize a CRSF Interface
 *
 * @param pHndl Blank handle to Initialize
 * @param pTx_hndl Transmit Stream Buffer (to send crsf packets to)
 * @return pRx_hndl - buffer the CRSF interface reads from
 */
extern StreamBufferHandle_t crsf_init(CRSF_t *pHndl,
                                      StreamBufferHandle_t pTx_hndl);

extern eCRSFError crsf_write_rc(CRSF_t *pHndl, const crsf_rc_t *pChannels);
extern eCRSFError crsf_write_battery(CRSF_t *pHndl, const crsf_battery_t *pBattery);
extern eCRSFError crsf_write_attitude(CRSF_t *pHndl,
                                      const crsf_attitude_t *pAttitude);
extern eCRSFError crsf_write_mode(CRSF_t *pHndl, const crsf_fcmode_t *pMode);

extern eCRSFError crsf_read_rc(CRSF_t *pHndl, crsf_rc_t *pChannels);
extern eCRSFError crsf_read_battery(CRSF_t *pHndl, crsf_battery_t *pBattery);
extern eCRSFError crsf_read_attitude(CRSF_t *pHndl, crsf_attitude_t *pAttitude);
extern eCRSFError crsf_read_mode(CRSF_t *pHndl, crsf_fcmode_t *pMode);

static inline float crsf_normalize(uint16_t raw){
    if (raw < CRSF_CHANNEL_MIN)
        raw = CRSF_CHANNEL_MIN;
    if (raw > CRSF_CHANNEL_MAX)
        raw = CRSF_CHANNEL_MAX;
    return ((float) raw - 992.0f) / 819.5f;
}

static inline uint16_t crsf_unnormal(float value){
    if (value < -1)
        value = -1;
    if (value > 1)
        value = 1;
    return (uint16_t)((value*819.5f) + 992.0f);
}

#endif
