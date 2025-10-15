/**
 * @file csrf_types.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2025-10-14
 * @modified Last Modified: 2025-10-14
 *
 * @copyright Copyright (c) 2025
 */

#ifndef _CRSF_TYPES_H_
#define _CRSF_TYPES_H_

#include <stdint.h>

#define CRSF_DATA_MAXLEN 4

typedef struct {
    uint8_t addr;
    uint8_t length;
    uint8_t type;
    uint8_t data[CRSF_DATA_MAXLEN];
    uint8_t crc;
} crsf_msg_t;

#endif
