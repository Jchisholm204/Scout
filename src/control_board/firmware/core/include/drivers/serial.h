/**
 * @file serial.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief UART Serial Driver
 * @version 0.1
 * @date 2024-10-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef _SERIAL_H_
#define _SERIAL_H_
#include "FreeRTOS.h"
#include "config/sys_cfg.h"
#include "hal/hal_uart.h"
#include "semphr.h"
#include "stream_buffer.h"

#include <stdio.h>

typedef enum {
    eSerialOK,
    // Hardware Error
    eSerialHW,
    // Semaphore Lock Timeout
    eSerialSemphr,
    // Initialization Failed
    eSerialInitFail,
    // The Serial interface has not been Initialized
    eSerialNoInit,
    // Serial interface is write locked
    eSerialLocked,
    // Null reference passed
    eSerialNULL
} eSerialError;

typedef enum {
    eSerial1,
    eSerial2,
    eSerial3,
    eSerial4,
    eSerial5,
    eSerial6,
    eSerialN
} eSerial;

typedef struct Serial {
    USART_TypeDef *UART;
    IRQn_Type IRQn;
    SemaphoreHandle_t tx_hndl;
    StaticSemaphore_t static_tx_semphr;
    StreamBufferHandle_t *rx_buf;
    eSerialError state;
} Serial_t;

/**
 * @brief Initialize a Serial Interface
 *
 * @param UART Handle
 * @param dev UART device to Initialize
 * @param pin_rx RX Pin
 * @param pin_tx TX Pin
 * @returns Serial
 */
extern Serial_t *serial_init(eSerial serial,
                             unsigned long baud,
                             pin_t pin_rx,
                             pin_t pin_tx);

/**
 * @brief Write to a Serial Interface (BLOCKING)
 *
 * @param pHndl Handle to write to
 * @param buf buffer to write
 * @param len length of buffer (in bytes)
 * @param timeout timeout to wait for semaphore
 * @returns eSerialOK or Error
 */
extern eSerialError serial_write(Serial_t *pHndl,
                                 char *buf,
                                 size_t len,
                                 TickType_t timeout);


/**
 * @brief Sttach an RX buffer to a Serial Interface
 *
 * @param pHndl Serial Interface to attach the RX buffer to
 * @param pBuf Pointer to the Serial Buffer Structure
 * @returns eSerialOK or Error
 */
extern eSerialError serial_attach(Serial_t *pHndl,
                                  StreamBufferHandle_t *buf_hndl);

/**
 * @brief detach an RX buffer to a Serial Interface
 *
 * @param pHndl Serial Interface to detach from
 * @returns eSerialOK or Error
 */
extern eSerialError serial_detach(Serial_t *pHndl);


#endif
