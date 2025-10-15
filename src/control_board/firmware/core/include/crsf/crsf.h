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

typedef enum {
    eCSRFOK
} eCRSFError;

typedef struct CRSF {
    Serial_t* pSerial;

    // Task information (Maybe not needed)
    TaskHandle_t tsk_hndl;
    StaticTask_t tsk_buf;
    StackType_t  tsk_stack[configMINIMAL_STACK_SIZE];

    // Recieve Buffer (from serial driver interrupt)
    StreamBufferHandle_t rx_hndl;
    StaticStreamBuffer_t rx_streamBuf;
    uint8_t rx_buf[configMINIMAL_STACK_SIZE];

    // Send buffer (to task)
    StreamBufferHandle_t tx_hndl;
    StaticStreamBuffer_t tx_streamBuf;
    crsf_msg_t tx_buf[configMINIMAL_STACK_SIZE];
    TaskHandle_t tx_owner;

    eCRSFError state;
} CRSF_t;

extern eCRSFError crsf_init(CRSF_t* pHndl, Serial_t* pSerial);
extern eCRSFError crsf_attach(CRSF_t *pHndl, StreamBufferHandle_t *buf_hndl);
extern eCRSFError crsf_detatch(CRSF_t *pHndl);
extern eCRSFError crsf_write(CRSF_t *pHndl, crsf_msg_t *pMsg);

#endif
