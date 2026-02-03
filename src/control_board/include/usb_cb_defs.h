/**
 * @file usb_cb_defs.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief 
 * @version 0.1
 * @date Created: 2026-01-11
 * @modified Last Modified: 2026-01-11
 *
 * @copyright Copyright (c) 2026
 */

#ifndef _USB_CB_DEFS_H_
#define _USB_CB_DEFS_H_

enum eCBMode {
    eModeDisabled,
    eModeInit,
    eModeRC,
    eModeRCAuto,
    eModeAuto,
    eModeStalled,
    eModeFault
};

enum eCBStatus {
    eStatusOK,
    eStatusFault,
    eStatusBlocked,
};

enum eCBFault {
    eFaultOK = 0U,
    eFaultLiDAR = 1U,
    eFaultCRSF = 2U,
    eFaultUSB = 4U,
    eFaultCollision = 8U,
};

enum eCBLidar {
    eLidarFront = 0U,
    eLidarVertical = 1U
};

#endif
