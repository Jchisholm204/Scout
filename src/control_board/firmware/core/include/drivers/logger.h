/**
 * @file logger.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief FreeRTOS based logging utility
 * @version 0.1
 * @date Created: 2025-10-16
 * @modified Last Modified: 2025-10-16
 *
 * @copyright Copyright (c) 2025
 */

#ifndef _LOGGER_H_
#define _LOGGER_H_

enum eLogType{
    eLogTrace,
    eLogDebug,
    eLogInfo,
    eLogWarn,
    eLogError
};

void logger_init(void);



#endif
