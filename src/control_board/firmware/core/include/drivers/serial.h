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
#include "hal/hal_uart.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stream_buffer.h"
#include "config/sys_cfg.h"
#include <stdio.h>

enum eSerialError {
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
};

typedef enum eSerialError eSerialError;

typedef struct Serial {
    USART_TypeDef *UART;
    IRQn_Type IRQn;
    FILE *fp;
    uint32_t tx_lock;
    SemaphoreHandle_t tx_hndl;
    StaticSemaphore_t static_tx_semphr;
    StreamBufferHandle_t *rx_buf;
    eSerialError state;
} Serial_t;


/**
 * @brief Initialize a Serial Interface
 *
 * @param pHndl Pointer to Serial Handle
 * @param dev UART device to Initialize
 * @param pin_rx RX Pin
 * @param pin_tx TX Pin
 * @returns eSerialOK or Error
 */
extern eSerialError serial_init(Serial_t *pHndl, unsigned long baud, pin_t pin_rx, pin_t pin_tx);

/**
 * @brief Write Lock a serial interface (only allow it to be written to by tasks with a lock_code)
 * Must use serial_write_locked instead of serial_write for locked interfaces
 *
 * @param pHndl Handle to lock
 * @param lock_code Locking code to use
 * @return serial error or eSerialOK
 */
extern eSerialError serial_lock(Serial_t *pHndl, uint32_t lock_code);

/**
 * @brief Unlock a locked serial interface
 *
 * @param pHndl Handle to unlock
 * @param lock_code Lock code used when the interface was locked
 * @return 
 */
extern eSerialError serial_unlock(Serial_t *pHndl, uint32_t lock_code);

/**
 * @brief Write to a Serial Interface (BLOCKING)
 *
 * @param pHndl Handle to write to
 * @param buf buffer to write
 * @param len length of buffer (in bytes)
 * @param timeout timeout to wait for semaphore
 * @returns eSerialOK or Error
 */
extern eSerialError serial_write(Serial_t *pHndl, char *buf, size_t len, TickType_t timeout);

/**
 * @brief Write to a Locked Serial Interface (BLOCKING)
 *  Must be used when a serial interface is locked.
 *  Will also work to write to unlocked interfaces
 *
 * @param pHndl Handle to write to
 * @param buf buffer to write
 * @param len length of buffer (in bytes)
 * @param timeout timeout to wait for semaphore
 * @param lock_code The code used to lock the serial interface
 * @returns eSerialOK or Error
 */
extern eSerialError serial_write_locked(Serial_t *pHndl, char *buf, size_t len, TickType_t timeout, uint32_t lock_code);

/**
 * @brief Sttach an RX buffer to a Serial Interface
 *
 * @param pHndl Serial Interface to attach the RX buffer to
 * @param pBuf Pointer to the Serial Buffer Structure
 * @returns eSerialOK or Error
 */
extern eSerialError serial_attach(Serial_t *pHndl, StreamBufferHandle_t *buf_hndl);

/**
 * @brief detach an RX buffer to a Serial Interface
 *
 * @param pHndl Serial Interface to detach from
 * @returns eSerialOK or Error
 */
extern eSerialError serial_detach(Serial_t *pHndl);


#if (configUSE_SERIAL1 == 1)
extern void USART1_IRQHandler(void);
extern Serial_t Serial1;
#endif
#if (configUSE_SERIAL2 == 1)
extern void USART2_IRQHandler(void);
extern Serial_t Serial2;
#endif
#if (configUSE_SERIAL3 == 1)
extern void USART3_IRQHandler(void);
extern Serial_t Serial3;
#endif
#if (configUSE_SERIAL4 == 1)
extern void UART4_IRQHandler(void);
extern Serial_t Serial4;
#endif
#if (configUSE_SERIAL5 == 1)
extern void UART5_IRQHandler(void);
extern Serial_t Serial5;
#endif
#if (configUSE_SERIAL6 == 1)
extern void USART6_IRQHandler(void);
extern Serial_t Serial6;
#endif

#endif

