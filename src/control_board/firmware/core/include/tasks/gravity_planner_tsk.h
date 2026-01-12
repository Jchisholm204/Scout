/**
 * @file gravity_planner_tsk.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2025-10-17
 * @modified Last Modified: 2025-10-17
 *
 * @copyright Copyright (c) 2025
 */

#ifndef _GRAVITY_PLANNER_H_
#define _GRAVITY_PLANNER_H_
#include "FreeRTOS.h"
#include "config/sys_cfg.h"
#include "protocols/crsf/crsf.h"
#include "drone_defs.h"
#include "protocols/rplidar/rplidar.h"
#include "semphr.h"
#include "stream_buffer.h"

#include <stdio.h>

typedef enum {
    eGPOK
} eGPlanErrror;

typedef struct {

    // Task information
    TaskHandle_t tsk_hndl;
    StaticTask_t tsk_buf;
    StackType_t tsk_stack[configMINIMAL_STACK_SIZE];

    StreamBufferHandle_t* tx_hndl;

    // XY Lidar
    RpLidar_t *lxy_pLidar;
    RpLidarScan lxy_scan;

    // ZX Lidar
    RpLidar_t *lzx_pLidar;
    RpLidarScan lzx_scan;

} GPlan_t;

/**
 * @brief Init the Gravity Planner Task
 *
 * @param pHndl Gravity Planner Instance Handle
 * @param pLidarXY Lidar instance that scans the XY plane
 * @param pLidarZX Lidar instance that scans the ZX plane
 * @return 
 */
extern eGPlanErrror gplan_init(GPlan_t* pHndl,
                               RpLidar_t* pLidarXY,
                               RpLidar_t* pLidarZX);


/**
 * @brief Register a notifier callback for when gplan computes the collision velocity
 *
 * @param pHndl Gravity Planner Handle
 * @param notify_tskHndl Handle of task to notify
 * @return 
 */
extern eGPlanErrror gplan_notify(GPlan_t *pHndl, TaskHandle_t *notify_tskHndl);

/**
 * @brief Read the latest collision velocity
 *
 * @param pHndl Gravity Planner Handle
 * @param pVel Velocity to execute to avoid collision
 * @return 
 */
extern eGPlanErrror gplan_read(GPlan_t *pHndl, drone_pos_t *pVel);

#endif
