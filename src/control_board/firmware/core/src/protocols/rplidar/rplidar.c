/**
 * @file rplidar.c
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2025-10-17
 * @modified Last Modified: 2025-10-17
 *
 * @copyright Copyright (c) 2025
 */

#include "protocols/rplidar/rplidar.h"
#include "os/systime.h"
#include "string.h"

void vRpLidar_tsk(void* pvParams);

eRpLidarError rplidar_init(RpLidar_t* pHndl,
                           Serial_t* pSerial,
                           pin_t stx,
                           pin_t srx) {

    // Perform initial checks
    if (!pHndl)
        return eRpLidarNULL;
    if (!pSerial)
        return eRpLidarNULL;

    pHndl->pSerial = pSerial;

    // designate memory for buffer
    pHndl->rx_hndl = xStreamBufferCreateStatic(RPLIDAR_BUF_LEN,
                                               1 /*!!change this!!*/,
                                               pHndl->rx_buf,
                                               &pHndl->rx_streamBuf);
    if (!pHndl->rx_hndl) {
        pHndl->state = eRpLidarInitFail;
        return pHndl->state;
    }

    // tell serial interface that the buffer exists
    if (serial_attach(pHndl->pSerial, pHndl->rx_hndl) != eSerialOK) {
        printf("cannot attach Serial3\n");
    }


    // Init RpLidar Task
    pHndl->tsk_hndl = xTaskCreateStatic(vRpLidar_tsk,
                                        "RPLDR",
                                        configMINIMAL_STACK_SIZE,
                                        (void*) pHndl,
                                        configMAX_PRIORITIES - 3,
                                        pHndl->tsk_stack,
                                        &pHndl->tsk_buf);
    if (!pHndl->tsk_hndl) {
        pHndl->state = eRpLidarTskCreateFail;
        return pHndl->state;
    }

    // Initialize LiDAR Device

    return eRpLidarOK;
}

eRpLidarError rplidar_notify(RpLidar_t* pHndl, TaskHandle_t* const pNotify_tskHndl) {
    if (!pHndl)
        return eRpLidarNULL;
    if (!pNotify_tskHndl)
        return eRpLidarNULL;
#warning "rplidar_notify not implimented"
    return eRpLidarOK;
}

eRpLidarError rplidar_read(RpLidar_t* pHndl, RpLidarScan* const pScan) {
    (void)pHndl;
    (void)pScan;
#warning "rplidar_read not implimented"
    // 1. Aquire read lock
    // 2. Memcpy to the dest
    // 3. Release read lock
    return eRpLidarOK;
}

void vRpLidar_tsk(void* pvParams){
    RpLidar_t* pHndl = pvParams;
    if (!pHndl) {
        vTaskSuspend(NULL);
    }
    // send a GET_INFO request
    // output to stdout in debug mode to see sent request
    // wait
    // receive GET_INFO response descriptor
    // output to stdout in debug mode to see response
    // wait
    // receive GET_INFO data response
    // output to stdout in debug mode to see response
    
    char request[16];
    uint8_t response[32];
    request[0] = 0xA5;
    request[1] = 0x50;
    if (serial_write(pHndl->pSerial, request, 2, 10) != eSerialOK){
        printf("cannot write GET_INFO request to Serial3\n");
    }
    printf("wrote GET_INFO request to Serial3\n");

    printf("reading from Serial3:\n");
    for (int i = 0; i < 7;) {
        if (xStreamBufferReceive(pHndl->rx_hndl, response+i, 1, 10)) {
            printf("byte %d: %X\n", i, response[i]);
            i++;
        }
    }
    
    if (!(response[0]==RPLIDAR_RESPONSE_DESCRIPTOR.start_flag1 && response[1]==RPLIDAR_RESPONSE_DESCRIPTOR.start_flag2)) {
        printf("format error on RPLidar response");
    }
    printf("GET_INFO response descriptor: %X %X %X %X %X %X %X\n", response[0], response[1], response[2], response[3], response[4], response[5], response[6]);

    RpLidarResponseDescriptor get_info_response_descriptor;
    memcpy(&get_info_response_descriptor, response, sizeof(RpLidarResponseDescriptor));
    // TODO: print stuff from get_info_response_descriptor to make sure we are parsing correctly

    uint32_t data_response_length = get_info_response_descriptor.data_response_detail & 0xFFFFFFFC;
    for (uint32_t i = 0; i < data_response_length;) {
        if (xStreamBufferReceive(pHndl->rx_hndl, response+i, 1, 10)) {
            printf("byte %lu: %X\n", i, response[i]);
            i++;
        }
    }

    // send a GET_HEALTH request
    // wait
    // receive GET_HEALTH response descriptor
    // wait
    // receive GET_HEALTH 


    
    TickType_t last_wake_time = xTaskGetTickCount();
    for (;;) {
        printf("vRpLidar_tsk\n");
        vTaskDelayUntil(&last_wake_time, 1000);
    }
}
