/**
 * @file crsf_internal.h
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2025-10-18
 * @modified Last Modified: 2025-10-18
 *
 * @copyright Copyright (c) 2025
 */

#ifndef _CRSF_INT_H_
#define _CRSF_INT_H_
#ifdef CRSF_INTERNAL
#include "crsf.h"
#include "crsf_types_internal.h"

#include <stdint.h>

#define CRSF_ADDR 0xC8

/**
 * @brief CRSF internal transmit packet function
 *
 * @param pSerial Serial device to transmit on
 * @param len Length of packet (only payload segment)
 * @param type Packet type (ID)
 * @param pData Pointer to payload data
 * @return 
 */
extern eCRSFError _crsf_send_packet(Serial_t* pSerial,
                                    uint8_t len,
                                    enum eCRSFMsgId type,
                                    uint8_t* pData);


/**
 * @brief Performs logic to check if a packet is valid and return its unpacked form
 *
 * @param pIn compressed packet from off the wire
 * @param pOut expanded packet for external use
 * @return 
 */
extern eCRSFError _crsf_recv_packet(_crsf_msg_t* pIn, crsf_msg_t *pOut);

extern uint8_t _crsf_crc8(const uint8_t* ptr, uint8_t len);

#endif
#endif
