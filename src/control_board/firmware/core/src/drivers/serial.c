/**
 * @file serial.c
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief UART Serial Driver
 * @version 0.1
 * @date 2024-10-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "drivers/serial.h"

#include "config/nvic.h"

static Serial_t SerialPort[eSerialN] = {
    {USART1, USART1_IRQn, NULL, {0}, NULL, NULL, eSerialNoInit},
    {USART2, USART2_IRQn, NULL, {0}, NULL, NULL, eSerialNoInit},
    {USART3, USART3_IRQn, NULL, {0}, NULL, NULL, eSerialNoInit},
    {UART4, UART4_IRQn, NULL, {0}, NULL, NULL, eSerialNoInit},
    {UART5, UART5_IRQn, NULL, {0}, NULL, NULL, eSerialNoInit},
    {USART6, USART6_IRQn, NULL, {0}, NULL, NULL, eSerialNoInit},
};

Serial_t *serial_init(eSerial serial,
                      unsigned long baud,
                      pin_t pin_rx,
                      pin_t pin_tx) {
    if (serial >= eSerialN) {
        return NULL;
    }

    // Retrive handle and ensure handle is not initialized
    Serial_t *pHndl = &SerialPort[serial];
    if (pHndl->state == eSerialOK)
        return NULL;

    hal_uart_init(pHndl->UART, baud, pin_tx, pin_rx);

    pHndl->tx_hndl = xSemaphoreCreateMutexStatic(&pHndl->static_tx_semphr);
    if (pHndl->tx_hndl == NULL) {
        pHndl->state = eSerialInitFail;
        return NULL;
    }

    xSemaphoreGive(pHndl->tx_hndl);
    pHndl->state = eSerialOK;

    pHndl->tx_buf = NULL;
    pHndl->rx_buf = NULL;

    return pHndl;
}

StreamBufferHandle_t serial_create_write_buffer(
    Serial_t *pSerial,
    size_t xBufferSizeBytes,
    size_t xTriggerLevelBytes,
    uint8_t *const pucStreamBufferStorageArea,
    StaticStreamBuffer_t *const pxStaticStreamBuffer) {
    if(!pSerial){
        return NULL;
    }
    StreamBufferHandle_t hndl =
        xStreamBufferCreateStaticWithCallback(xBufferSizeBytes,
                                              xTriggerLevelBytes,
                                              pucStreamBufferStorageArea,
                                              pxStaticStreamBuffer,
                                              vSerial_RxCallback,
                                              NULL);
    if(!hndl){
        return NULL;
    }
    pSerial->tx_buf = hndl;
    NVIC_EnableIRQ(pSerial->IRQn);
    NVIC_SetPriority(pSerial->IRQn, NVIC_Priority_MIN);
    return hndl;
}

eSerialError serial_write(Serial_t *pHndl,
                          char *buf,
                          size_t len,
                          TickType_t timeout) {
    if (pHndl == NULL || buf == NULL)
        return eSerialNULL;
    if (pHndl->state != eSerialOK)
        return pHndl->state;
    if (xSemaphoreTake(pHndl->tx_hndl, timeout) == pdTRUE) {
        hal_uart_write_buf(pHndl->UART, buf, len);
        xSemaphoreGive(pHndl->tx_hndl);
        return eSerialOK;
    }
    return eSerialSemphr;
}

eSerialError serial_attach(Serial_t *pHndl, StreamBufferHandle_t buf_hndl) {
    if (pHndl == NULL || buf_hndl == NULL)
        return eSerialNULL;
    if (pHndl->state != eSerialOK)
        return pHndl->state;
    if (pHndl->rx_buf == NULL) {
        pHndl->rx_buf = buf_hndl;
        NVIC_EnableIRQ(pHndl->IRQn);
        NVIC_SetPriority(pHndl->IRQn, NVIC_Priority_MIN);
        hal_uart_enable_rxne(pHndl->UART, true);
        return eSerialOK;
    }
    return eSerialSemphr;
}

eSerialError serial_detach(Serial_t *pHndl) {
    if (pHndl == NULL)
        return eSerialNULL;
    if (pHndl->state != eSerialOK)
        return pHndl->state;
    pHndl->rx_buf = NULL;
    hal_uart_enable_rxne(pHndl->UART, false);
    if (!pHndl->tx_buf) {
        NVIC_DisableIRQ(pHndl->IRQn);
    }
    return eSerialOK;
}

void vSerial_RxCallback(StreamBufferHandle_t xStreamBuffer,
                        BaseType_t xIsInsideISR,
                        BaseType_t *const pxHigherPriorityTaskWoken) {
    (void) xStreamBuffer;
    (void) xIsInsideISR;
    (void) pxHigherPriorityTaskWoken;
    // Enable all active ports with full buffers
    for (int i = 0; i < eSerialN; i++) {
        // Enable the TX interrupt if the serial state is OK and there is a TX
        // buffer attached, and that buffer contains data elements to be read
        if (SerialPort[i].state == eSerialOK && SerialPort[i].tx_buf != NULL) {

            size_t bytes_avail =
                xStreamBufferBytesAvailable(SerialPort[i].tx_buf);
            if (bytes_avail > 0)
                hal_uart_enable_txne(SerialPort[i].UART, true);
        }
    }
}

void generic_handler(Serial_t *pHndl) {
    // MUST read status to clear ORE/NF/FE flags
    uint32_t status = hal_uart_read_status(pHndl->UART);
    BaseType_t higher_woken = pdFALSE;
    // Interrupt Generated for Read
    if (status & USART_SR_RXNE) {
        // MUST read input port to clear iPending bit
        uint8_t rx_data = hal_uart_read_byte(pHndl->UART);
        // hal_uart_write_byte(pHndl->UART, rx_data);
        // Check that a handler exists
        if (pHndl->rx_buf == NULL || pHndl->state != eSerialOK) {
            // If there is no handler disable this interrupt
            hal_uart_enable_rxne(pHndl->UART, false);
            return;
        }
        // If a buffer exists, run the interrupt routine
        xStreamBufferSendFromISR(
            pHndl->rx_buf, &rx_data, sizeof(rx_data), &higher_woken);
    }
    if (status & USART_SR_TXE) {
        if (pHndl->tx_buf == NULL || pHndl->state != eSerialOK) {
            // If there is no handler disable this interrupt
            hal_uart_enable_txne(pHndl->UART, false);
            return;
        }
        uint8_t tx_data = 0;
        if (xStreamBufferReceiveFromISR(
                pHndl->tx_buf, &tx_data, 1, &higher_woken) == 1) {
            hal_uart_write_byte(pHndl->UART, tx_data);
        } else {
            hal_uart_enable_txne(pHndl->UART, false);
        }
        // if (xStreamBufferBytesAvailable(pHndl->tx_buf) == 0) {
        // }
    }
    portYIELD_FROM_ISR(higher_woken);
}

void USART1_IRQHandler(void) {
    generic_handler(&SerialPort[eSerial1]);
}

void USART2_IRQHandler(void) {
    generic_handler(&SerialPort[eSerial2]);
}

void USART3_IRQHandler(void) {
    generic_handler(&SerialPort[eSerial3]);
}

void UART4_IRQHandler(void) {
    generic_handler(&SerialPort[eSerial4]);
}

void UART5_IRQHandler(void) {
    generic_handler(&SerialPort[eSerial5]);
}

void USART6_IRQHandler(void) {
    generic_handler(&SerialPort[eSerial6]);
}
