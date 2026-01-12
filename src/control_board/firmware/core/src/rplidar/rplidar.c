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

#define RPLIDAR_BAUD 1234
#define RPLIDAR_SLOCK 0x7787

void vRpLidar_tsk(void* pvParams);

eRpLidarError rplidar_init(RpLidar_t* pHndl,
                           Serial_t* pSerial,
                           pin_t stx,
                           pin_t srx) {
#warning "RpLiDAR Task not completed"

    // Perform initial checks
    if (!pHndl)
        return eRpLidarNULL;
    if (!pSerial)
        return eRpLidarNULL;
    if (pSerial->state != eSerialNoInit) {
        pHndl->state = eRpLidarInitFail;
        return pHndl->state;
    }

    // Initialize Self
    pHndl->pSerial = pSerial;
    eSerialError se = eSerialOK;
    if ((se = serial_init(pSerial, RPLIDAR_BAUD, srx, stx)) != eSerialOK) {
        pHndl->state = eRpLidarSerialFail;
        return pHndl->state;
    }

    // Ensure nothing else can write to the CRSF serial port
    se = serial_lock(pSerial, RPLIDAR_SLOCK);
    if (se != eSerialOK)
        return eRpLidarSerialFail;

    pHndl->rx_hndl = xStreamBufferCreateStatic(RPLIDAR_BUF_LEN,
                                               1 /*!!change this!!*/,
                                               pHndl->rx_buf,
                                               &pHndl->rx_streamBuf);
    if (!pHndl->rx_hndl) {
        pHndl->state = eRpLidarInitFail;
        return pHndl->state;
    }

    se = serial_attach(pSerial, &pHndl->rx_hndl);
    if (se != eSerialOK) {
        pHndl->state = eRpLidarSerialFail;
        return pHndl->state;
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

void vRpLidar_tsk(void* pvParams);
