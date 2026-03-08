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
#include "usb_lidar.h"
#include "usb_packet.h"
#include <assert.h>

void vRpLidar_tsk(void* pvParams);

eRpLidarError rplidar_init(RpLidar_t* pHndl,
                           uint8_t id,
                           Serial_t* pSerial,
                           QueueHandle_t tx/*,
                           pin_t stx,
                           pin_t srx*/) {

    // Perform initial checks
    if (!pHndl)
        return eRpLidarNULL;
    if (!pSerial)
        return eRpLidarNULL;

    pHndl->id = id;
    pHndl->pSerial = pSerial;
    pHndl->tx = tx;

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
                                        RPLIDAR_STACK_SIZE,
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

#define max(a, b) (((a) > (b)) ? (a) : (b))
#define min(a, b) (((a) < (b)) ? (a) : (b))

#if 0
static inline void print_bytes(const void *data, size_t len) {
    const uint8_t *p = data;
    for (size_t i = 0; i < len; i++)
        printf("byte %u: %02X\n", i, p[i]);
}
#endif


static eRpLidarError stream_read_exact(
    StreamBufferHandle_t stream,
    void *dst,
    size_t len,
    TickType_t timeout) {

    uint8_t *p = dst;
    TickType_t start = xTaskGetTickCount();

    for (size_t total = 0, r; total < len; total += r) {
        TickType_t remaining = timeout - (xTaskGetTickCount() - start);

        if (remaining <= 0)
            return eRpLidarSerialFail;

        r = xStreamBufferReceive(
            stream,
            p + total,
            len - total,
            remaining
        );

        if (r == 0)
            return eRpLidarSerialFail; 
    }

    return eRpLidarOK;
}

#if 0
static eRpLidarError stream_read_until(
    StreamBufferHandle_t stream,
    void *dst,
    size_t len,
    uint8_t terminator,
    TickType_t timeout) {

    uint8_t *p = dst;
    TickType_t start = xTaskGetTickCount();

    for (size_t total = 0, r; (total < len) && p[total]!=terminator; total += r) {
        TickType_t remaining = timeout - (xTaskGetTickCount() - start);

        if (remaining <= 0)
            return eRpLidarSerialFail;

        r = xStreamBufferReceive(
            stream,
            p + total,
            1,
            remaining
        );

        if (r == 0)
            return eRpLidarSerialFail; 
    }

    return eRpLidarOK;
}

static eRpLidarError stream_write_exact(
    StreamBufferHandle_t stream,
    const void *src,
    size_t len,
    TickType_t timeout)
{
    const uint8_t *p = src; 

    TickType_t start = xTaskGetTickCount();

    for (size_t total = 0, w; total < len; total += w)
    {
        TickType_t remaining = timeout - (xTaskGetTickCount() - start);

        if (remaining <= 0)
            return eRpLidarStreamFail;

        w = xStreamBufferSend(
            stream,
            p + total,
            len - total,
            remaining
        );

        if (w == 0)
            return eRpLidarStreamFail;
    }

    return eRpLidarOK;
}
#endif

static inline void print_udev_pkt_lidar(uint16_t indent, struct udev_pkt_lidar const* const pkt) {
    printf("%*sid: %01X\n", indent, "", pkt->hdr.id);
    printf("%*ssequence: 0x%02X\n", indent, "", pkt->hdr.sequence);
    printf("%*slen: 0x%02X\n", indent, "", pkt->hdr.len);
    printf("%*sdistances_sum: 0x%04X\n", indent, "", pkt->distance_sum);
    const uint16_t *distances = pkt->distances;
    printf("%*sdistances: 0x%04X\n", indent, "", distances[0]);
    uint8_t i = 0;
    do {
        printf("%*s         : 0x%04X\n", indent, "", distances[i]);
    } while (++i < sizeof(pkt->distances)/sizeof(pkt->distances[0]));
}

static inline uint8_t checksum_RpLidarRequestPacketWithPayload(RpLidarRequestWithPayload *request_packet) {
    uint8_t checksum = 0 ^ (request_packet->start_flag) ^ (request_packet->command) ^ (request_packet->payload_size);
    uint8_t *payload = request_packet->payload, i = 0, payload_size = request_packet->payload_size;
    do {
        checksum ^= payload[i];
    } while (++i < payload_size);
    return checksum;
}

static inline uint16_t memcpy_RpLidarRequestPacketWithPayload(char *dst, const RpLidarRequestWithPayload *pkt) {
    memcpy(dst, pkt, offsetof(typeof(*pkt), payload));
    memcpy(dst+offsetof(typeof(*pkt), payload), pkt->payload, pkt->payload_size);
    memcpy(dst+offsetof(typeof(*pkt), payload) + pkt->payload_size, &(pkt->checksum), sizeof(pkt->checksum));
    return sizeof(*pkt) - sizeof(pkt->payload) + (pkt->payload_size);
}

void vRpLidar_tsk(void* pvParams){
    RpLidar_t* pHndl = pvParams;
    if (!pHndl) {
        vTaskSuspend(NULL);
    }
    QueueHandle_t tx = pHndl->tx;
    TickType_t last_wake_time = xTaskGetTickCount();
    const char *request_name;
    RpLidarRequest request_packet_no_payload = {.start_flag=START_FLAG};
    RpLidarRequestWithPayload request_packet_with_payload;
    RpLidarResponseDescriptor response_descriptor;
    char request[32];
    uint16_t request_size;
    // RpLidarExpressScanDataResponse express_scan_data_response;

    #define READ(dst, len) stream_read_exact(pHndl->rx_hndl, dst, len, 10)
    #define WRITE(buf, len) serial_write(pHndl->pSerial, (char *)buf, len, 10) 

    // GET_INFO
    request_name = "GET_INFO";
    // prepare GET_INFO request
    // send GET_INFO request
    // receive GET_INFO response descriptor
    // receive GET_INFO data response
    
    // prepare request
    request_packet_no_payload.command=COMMAND_GET_INFO;
    // print to make sure it's prepared correctly
    // printf("%s: request packet: \n", request_name);
    // print_RpLidarRequestNoPayload(4, request_packet_no_payload);
    // send request
    if (WRITE(&request_packet_no_payload, sizeof(request_packet_no_payload)) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // receive response descriptor
    if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
        printf("ERROR: cannot read GET_INFO response descriptor\n");
    if (!(response_descriptor.start_flag1==START_FLAG1 && response_descriptor.start_flag2==START_FLAG2))
        printf("ERROR: invalid format on RPLidar response\n");
    // print stuff to make sure we are parsing correctly
    // printf("%s: response descriptor:\n", request_name);
    // print_RpLidarResponseDescriptor(4, response_descriptor);
    if (response_descriptor.data_response_length != sizeof(RpLidarDeviceInfo))
        printf("ERROR: %s: response_descriptor.data_response_length != sizeof(RpLidarDeviceInfo)\n", request_name);
    // receive data response
    RpLidarDeviceInfo device_info;
    // printf("%s: reading %u bytes from Serial3:\n", request_name, sizeof(device_info));
    if (READ(&device_info, response_descriptor.data_response_length) != eRpLidarOK)
        printf("ERROR: %s: cannot read data response\n", request_name);
    // print stuff to make sure we are parsing correctly
    printf("%s: data response:\n", request_name);
    print_RpLidarDeviceInfo(4, device_info);

    // GET_HEALTH
    request_name = "GET_HEALTH";
    // prepare request
    // send a request
    // receive response descriptor
    // receive data response

    // prepare GET_HEALTH request
    request_packet_no_payload.command = COMMAND_GET_HEALTH;
    // print to make sure it's prepared correctly
    // printf("%s: request packet:\n", request_name);
    // print_RpLidarRequestNoPayload(4, request_packet_no_payload);
    // send a GET_HEALTH request
    if (WRITE(&request_packet_no_payload, sizeof(request_packet_no_payload)) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // receive GET_HEALTH response descriptor
    if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
        printf("ERROR: %s: cannot read response descriptor from Serial3\n", request_name);
    if (!(response_descriptor.start_flag1==START_FLAG1 && response_descriptor.start_flag2==START_FLAG2))
        printf("ERROR: %s: invalid format on RPLidar response\n", request_name);
    // print stuff to make sure we are parsing correctly
    // printf("%s: response descriptor:\n", request_name);
    // print_RpLidarResponseDescriptor(4, response_descriptor);
    if (response_descriptor.data_response_length != sizeof(RpLidarDeviceHealth))
        printf("ERROR: %s: response_descriptor.data_response_length != expected_data_response_length\n", request_name);
    // receive GET_HEALTH data response
    RpLidarDeviceHealth device_health;
    if (READ(&device_health, response_descriptor.data_response_length) != eRpLidarOK)
        printf("ERROR: %s: cannot read data response from Serial3\n", request_name);
    // print stuff to make sure we are parsing correctly
    printf("%s: data response:\n", request_name);
    print_RpLidarDeviceHealth(4, device_health);
    
    // GET_SAMPLERATE
    request_name = "GET_SAMPLERATE";
    // prepare request
    // send request
    // receive response descriptor
    // receive data response

    // prepare GET_SAMPLERATE request
    request_packet_no_payload.command = COMMAND_GET_SAMPLERATE;
    // printf("%s: request packet:\n", request_name);
    // print_RpLidarRequestNoPayload(4, request_packet_no_payload);
    // send request
    if (WRITE(&request_packet_no_payload, sizeof(request_packet_no_payload)) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // receive response descriptor
    if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
        printf("ERROR: %s: cannot read response descriptor from Serial3\n", request_name);
    if (!(response_descriptor.start_flag1==START_FLAG1 && response_descriptor.start_flag2==START_FLAG2))
        printf("ERROR: %s: invalid format on RPLidar response\n", request_name);
    // print stuff to make sure we are parsing correctly
    // printf("%s: response descriptor:\n", request_name);
    // print_RpLidarResponseDescriptor(4, response_descriptor);
    if (response_descriptor.data_response_length != sizeof(RpLidarSampleRate))
        printf("ERROR: %s: response_descriptor.data_response_length != expected_data_response_length\n", request_name);
    // receive GET_SAMPLERATE data response
    RpLidarSampleRate sample_rate;
    if (READ(&sample_rate, response_descriptor.data_response_length) != eRpLidarOK)
        printf("ERROR: %s: cannot read data response from Serial3\n", request_name);
    // print stuff to make sure we are parsing correctly
    printf("%s: data response:\n", request_name);
    print_RpLidarSampleRate(4, sample_rate);
    
    // GET_LIDAR_CONF
    // send GET_LIDAR_CONF requests for each configuration entry.
    // configuration entries: 0x70, 0x71, 0x74, 0x75, 0x7C, 0x7F.

    // RPLIDAR_CONF_SCAN_MODE_COUNT (0x70)
    request_name = "RPLIDAR_CONF_SCAN_MODE_COUNT";
    // prepare RPLIDAR_CONF_SCAN_MODE_COUNT request
    // send RPLIDAR_CONF_SCAN_MODE_COUNT request
    // receive RPLIDAR_CONF_SCAN_MODE_COUNT response descriptor
    // receive RPLIDAR_CONF_SCAN_MODE_COUNT data response

    // RPLIDAR returns the amount of scan modes supported when receives this command. 
    // RPLIDAR supports scan mode ids from 0 to (scan_mode_count – 1). 
    // For instance, device returning 2 according to this query means that the device support 2 work modes, whose ids are 0, 1. 
    // The host system may use the work mode id and other configuration type to get specific characters of the work mode.

    // prepare request packet
    request_packet_with_payload.command = COMMAND_GET_LIDAR_CONF;
    request_packet_with_payload.payload_size = sizeof(RpLidarConfRequestDataNoPayload);
    RpLidarConfRequestDataNoPayload request_data_no_payload = {.type=RPLIDAR_CONF_SCAN_MODE_COUNT};
    memcpy(request_packet_with_payload.payload, &request_data_no_payload, request_packet_with_payload.payload_size);
    request_packet_with_payload.checksum = checksum_RpLidarRequestPacketWithPayload(&request_packet_with_payload);
    request_size = memcpy_RpLidarRequestPacketWithPayload(request, &request_packet_with_payload);
    // printf("%s: request packet:\n", request_name);
    // print_RpLidarRequestWithPayload(4, &request_packet_with_payload);
    // send request packet
    if (WRITE(request, request_size) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // receive response descriptor
    while (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
        printf("ERROR: %s: cannot read response descriptor from Serial3\n", request_name);
    // print stuff to make sure we are parsing correctly
    // printf("%s: response descriptor:\n", request_name);
    // print_RpLidarResponseDescriptor(4, response_descriptor);
    if (response_descriptor.data_response_length != sizeof(RpLidarConfResponseDataWithU16Payload))
        printf("ERROR: %s: response_descriptor.data_response_length != expected_data_response_length\n", request_name);
    // receive data response
    RpLidarConfResponseDataWithU16Payload scan_mode_count_data_response;
    if (READ(&scan_mode_count_data_response, response_descriptor.data_response_length) != eRpLidarOK)
        printf("ERROR: %s: cannot read data response from Serial3\n", request_name);
    // print output
    printf("%s: data response:\n", request_name);
    print_RpLidarConfResponseDataWithU16Payload(4, scan_mode_count_data_response, "scan_mode_count");

    const uint16_t scan_mode_count = scan_mode_count_data_response.payload;
    
    // RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE (0x71)
    request_name = "RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE";
    // Get microsecond cost per measurement sample for specific scan mode (in Q8 fixed point format). Actual value = laser_range_time_q8/256.0 (microseconds).
    // prepare RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE request
    // send RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE request
    // receive RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE response descriptor
    // receive RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE data response
    RpLidarConfRequestDataWithU16Payload request_data_u16_payload;

    for (uint16_t mode = 0; mode < scan_mode_count; ++mode) {
        // prepare RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE request
        request_packet_with_payload.command = COMMAND_GET_LIDAR_CONF;
        request_packet_with_payload.payload_size = sizeof(RpLidarConfRequestDataWithU16Payload);
        request_data_u16_payload.type=RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE;
        request_data_u16_payload.payload=mode;
        memcpy(request_packet_with_payload.payload, &request_data_u16_payload, request_packet_with_payload.payload_size);
        request_packet_with_payload.checksum = checksum_RpLidarRequestPacketWithPayload(&request_packet_with_payload);
        request_size = memcpy_RpLidarRequestPacketWithPayload(request, &request_packet_with_payload);
        // printf("%s(%u): request packet:\n", request_name, mode);
        // print_RpLidarRequestWithPayload(4, &request_packet_with_payload);
        // send request packet
        if (WRITE(request, request_size) != eSerialOK)
            printf("ERROR: %s(%u): cannot write request packet to Serial3\n", request_name, mode);
        // receive response descriptor
        if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
            printf("ERROR: %s(%u): cannot read response descriptor from Serial3\n", request_name, mode);
        if (response_descriptor.data_response_length != sizeof(RpLidarConfResponseDataWithU32Payload))
            printf("ERROR: %s(%u): response_descriptor.data_response_length != expected_data_response_length\n", request_name, mode);
        // print stuff to make sure we are parsing correctly
        // printf("%s(%u): response descriptor:\n", request_name, mode);
        // print_RpLidarResponseDescriptor(4, response_descriptor);
        // receive data response
        RpLidarConfResponseDataWithU32Payload scan_mode_us_per_sample_data_response;
        if (READ(&scan_mode_us_per_sample_data_response, response_descriptor.data_response_length) != eRpLidarOK)
            printf("ERROR: %s(%u): cannot read data response from Serial3\n", request_name, mode);
        // print stuff to make sure we are parsing correctly
        printf("%s(%u): data response:\n", request_name, mode);
        print_RpLidarConfResponseDataWithU32Payload(4, scan_mode_us_per_sample_data_response, "laser_range_time_q8");
    }

    // RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE (0x74)
    request_name = "RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE";
    // Get max measurement distance for specific scan mode (in m, Q8 fixed point format). Actual Value = max_distance_q8/256.0 (meters).
    // prepare RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE request
    // send RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE request
    // receive RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE response descriptor
    // receive RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE data response

    for (uint16_t mode = 0; mode < scan_mode_count; ++mode) {
        // prepare request
        request_packet_with_payload.command = COMMAND_GET_LIDAR_CONF;
        request_packet_with_payload.payload_size = sizeof(RpLidarConfRequestDataWithU16Payload);
        request_data_u16_payload.type=RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE;
        request_data_u16_payload.payload=mode;
        memcpy(request_packet_with_payload.payload, &request_data_u16_payload, request_packet_with_payload.payload_size);
        request_packet_with_payload.checksum = checksum_RpLidarRequestPacketWithPayload(&request_packet_with_payload);
        request_size = memcpy_RpLidarRequestPacketWithPayload(request, &request_packet_with_payload);
        // printf("%s(%u): request packet:\n", request_name, mode);
        // print_RpLidarRequestWithPayload(4, &request_packet_with_payload);
        // send request packet
        if (WRITE(request, request_size) != eSerialOK)
            printf("ERROR: %s(%u): cannot write request packet to Serial3\n", request_name, mode);
        // receive response descriptor
        if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
            printf("ERROR: %s(%u): cannot read response descriptor from Serial3\n", request_name, mode);
        if (response_descriptor.data_response_length != sizeof(RpLidarConfResponseDataWithU32Payload))
            printf("ERROR: %s(%u): response_descriptor.data_response_length != expected_data_response_length\n", request_name, mode);
        // print stuff to make sure we are parsing correctly
        // printf("%s(%u): response descriptor:\n", request_name, mode);
        // print_RpLidarResponseDescriptor(4, response_descriptor);
        // receive data response
        RpLidarConfResponseDataWithU32Payload scan_mode_max_distance_data_response;
        if (READ(&scan_mode_max_distance_data_response, response_descriptor.data_response_length) != eRpLidarOK)
            printf("ERROR: %s(%u): cannot read data response from Serial3\n", request_name, mode);
        // print stuff to make sure we are parsing correctly
        printf("%s(%u): data response:\n", request_name, mode);
        print_RpLidarConfResponseDataWithU32Payload(4, scan_mode_max_distance_data_response, "max_distance_q8");
    }

    // RPLIDAR_CONF_SCAN_MODE_ANS_TYPE (0x75)
    request_name = "RPLIDAR_CONF_SCAN_MODE_ANS_TYPE";
    // prepare RPLIDAR_CONF_SCAN_MODE_ANS_TYPE request
    // send RPLIDAR_CONF_SCAN_MODE_ANS_TYPE request
    // receive RPLIDAR_CONF_SCAN_MODE_ANS_TYPE response descriptor
    // receive RPLIDAR_CONF_SCAN_MODE_ANS_TYPE data response

    for (uint16_t mode = 0; mode < scan_mode_count; ++mode) {
        // prepare request
        request_packet_with_payload.command = COMMAND_GET_LIDAR_CONF;
        request_packet_with_payload.payload_size = sizeof(RpLidarConfRequestDataWithU16Payload);
        request_data_u16_payload.type=RPLIDAR_CONF_SCAN_MODE_ANS_TYPE;
        request_data_u16_payload.payload=mode;
        memcpy(request_packet_with_payload.payload, &request_data_u16_payload, request_packet_with_payload.payload_size);
        request_packet_with_payload.checksum = checksum_RpLidarRequestPacketWithPayload(&request_packet_with_payload);
        request_size = memcpy_RpLidarRequestPacketWithPayload(request, &request_packet_with_payload);
        // printf("%s(%u): request packet:\n", request_name, mode);
        // print_RpLidarRequestWithPayload(4, &request_packet_with_payload);
        // send request packet
        if (WRITE(request, request_size) != eSerialOK)
            printf("ERROR: %s(%u): cannot write request packet to Serial3\n", request_name, mode);
        // receive response descriptor
        if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
            printf("ERROR: %s(%u): cannot read response descriptor from Serial3\n", request_name, mode);
        if (response_descriptor.data_response_length != sizeof(RpLidarConfResponseDataWithU8Payload))
            printf("ERROR: %s(%u): response_descriptor.data_response_length != expected_data_response_length\n", request_name, mode);
        // print stuff to make sure we are parsing correctly
        // printf("%s(%u): response descriptor:\n", request_name, mode);
        // print_RpLidarResponseDescriptor(4, response_descriptor);
        // receive data response
        RpLidarConfResponseDataWithU8Payload scan_mode_ans_type_data_response;
        if (READ(&scan_mode_ans_type_data_response, response_descriptor.data_response_length) != eRpLidarOK)
            printf("ERROR: %s(%u): cannot read data response from Serial3\n", request_name, mode);
        // print stuff to make sure we are parsing correctly
        printf("%s(%u): data response:\n", request_name, mode);
        print_RpLidarConfResponseDataWithU8Payload(4, scan_mode_ans_type_data_response, "ans_type");
    }

    // RPLIDAR_CONF_SCAN_MODE_TYPICAL (0x7C)
    request_name = "RPLIDAR_CONF_SCAN_MODE_TYPICAL";
    // prepare RPLIDAR_CONF_SCAN_MODE_TYPICAL request
    // send RPLIDAR_CONF_SCAN_MODE_TYPICAL request
    // receive RPLIDAR_CONF_SCAN_MODE_TYPICAL response descriptor
    // receive RPLIDAR_CONF_SCAN_MODE_TYPICAL data response 

    // prepare request packet
    request_packet_with_payload.command = COMMAND_GET_LIDAR_CONF;
    request_packet_with_payload.payload_size = sizeof(RpLidarConfRequestDataNoPayload);
    request_data_no_payload.type=RPLIDAR_CONF_SCAN_MODE_TYPICAL;
    memcpy(request_packet_with_payload.payload, &request_data_no_payload, request_packet_with_payload.payload_size);
    request_packet_with_payload.checksum = checksum_RpLidarRequestPacketWithPayload(&request_packet_with_payload);
    request_size = memcpy_RpLidarRequestPacketWithPayload(request, &request_packet_with_payload);
    // printf("%s: request packet:\n", request_name);
    // print_RpLidarRequestWithPayload(4, &request_packet_with_payload);
    // send request packet
    if (WRITE(request, request_size) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // receive response descriptor
    if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
        printf("ERROR: %s: cannot read response descriptor from Serial3\n", request_name);
    // print stuff to make sure we are parsing correctly
    // printf("%s: response descriptor:\n", request_name);
    // print_RpLidarResponseDescriptor(4, response_descriptor);
    if (response_descriptor.data_response_length != sizeof(RpLidarConfResponseDataWithU16Payload))
        printf("ERROR: %s: response_descriptor.data_response_length != expected_data_response_length\n", request_name);
    // receive data response
    RpLidarConfResponseDataWithU16Payload scan_mode_typical_data_response;
    if (READ(&scan_mode_typical_data_response, response_descriptor.data_response_length) != eRpLidarOK)
        printf("ERROR: %s: cannot read data response from Serial3\n", request_name);
    // print stuff to make sure we are parsing correctly
    printf("%s: data response:\n", request_name);
    print_RpLidarConfResponseDataWithU16Payload(4, scan_mode_typical_data_response, "scan_mode");

    // used for EXPRESS_SCAN
    const uint16_t typical_scan_mode = scan_mode_typical_data_response.payload;

    // RPLIDAR_CONF_SCAN_MODE_NAME
    request_name = "RPLIDAR_CONF_SCAN_MODE_NAME";
    // prepare RPLIDAR_CONF_SCAN_MODE_NAME request
    // send RPLIDAR_CONF_SCAN_MODE_NAME request
    // receive RPLIDAR_CONF_SCAN_MODE_NAME response descriptor
    // receive data response 

    for (uint16_t mode = 0; mode < scan_mode_count; ++mode) {
        // prepare request
        request_packet_with_payload.command = COMMAND_GET_LIDAR_CONF;
        request_packet_with_payload.payload_size = sizeof(RpLidarConfRequestDataWithU16Payload);
        request_data_u16_payload.type=RPLIDAR_CONF_SCAN_MODE_NAME;
        request_data_u16_payload.payload=mode;
        memcpy(request_packet_with_payload.payload, &request_data_u16_payload, request_packet_with_payload.payload_size);
        request_packet_with_payload.checksum = checksum_RpLidarRequestPacketWithPayload(&request_packet_with_payload);
        request_size = memcpy_RpLidarRequestPacketWithPayload(request, &request_packet_with_payload);
        // printf("%s(%u): request packet:\n", request_name, mode);
        // print_RpLidarRequestWithPayload(4, &request_packet_with_payload);
        // send request packet
        if (WRITE(request, request_size) != eSerialOK)
            printf("ERROR: %s(%u): cannot write request packet to Serial3\n", request_name, mode);
        // receive response descriptor
        if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
            printf("ERROR: %s(%u): cannot read response descriptor from Serial3\n", request_name, mode);
        if (response_descriptor.data_response_length > sizeof(RpLidarConfResponseDataWithStringPayload))
            printf("ERROR: %s(%u): response_descriptor.data_response_length > maximum_data_response_length\n", request_name, mode);
        // print stuff to make sure we are parsing correctly
        // printf("%s(%u): response descriptor:\n", request_name, mode);
        // print_RpLidarResponseDescriptor(4, response_descriptor);
        // receive data response
        RpLidarConfResponseDataWithStringPayload scan_mode_name_data_response;
        if (READ(&scan_mode_name_data_response, response_descriptor.data_response_length) != eRpLidarOK)
            printf("ERROR: %s(%u): cannot read data response from Serial3\n", request_name, mode);
        // print results
        printf("%s(%u): data response:\n", request_name, mode);
        print_RpLidarConfResponseDataWithStringPayload(4, scan_mode_name_data_response, "scan_mode_name");
    }

    // MOTOR_SPEED_CTRL
    // it seems like this has no effect. 
    request_name = "MOTOR_SPEED_CTRL";
    uint16_t motor_speed = 4000;
    // prepare request packet
    request_packet_with_payload.command = COMMAND_MOTOR_SPEED_CTRL;
    request_packet_with_payload.payload_size = sizeof(motor_speed);
    memcpy(request_packet_with_payload.payload, &motor_speed, sizeof(motor_speed));
    request_packet_with_payload.checksum = checksum_RpLidarRequestPacketWithPayload(&request_packet_with_payload);
    request_size = memcpy_RpLidarRequestPacketWithPayload(request, &request_packet_with_payload);
    // send request packet
    printf("%s: request packet:\n", request_name);
    print_RpLidarRequestWithPayload(4, &request_packet_with_payload);
    if (WRITE(request, request_size) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // done (there is no response)


    #if 0
    // SCAN
    request_name = "SCAN";
    // prepare SCAN request
    // send SCAN request
    // receive SCAN response descriptor
    // The data response packets related to every measurement sample results will be sent out continuously only after the motor rotation becomes stable.
    // receive SCAN data response packet, repeat as many times as you want.
    // send STOP request to stop scanning
    // discard any SCAN data response packets remaining in the receive buffer

    // prepare request packet
    request_packet_no_payload.command = COMMAND_SCAN;
    // print to make sure it's prepared correctly
    printf("%s: request packet: \n", request_name);
    print_RpLidarRequestNoPayload(4, request_packet_no_payload);
    // send request packet
    if (WRITE(&request_packet_no_payload, sizeof(request_packet_no_payload)) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // print stuff so we know what was sent
    // receive response descriptor
    if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
        printf("ERROR: cannot read response descriptor\n");
    if (!(response_descriptor.start_flag1==START_FLAG1 && response_descriptor.start_flag2==START_FLAG2))
        printf("ERROR: invalid format on RPLidar response\n");
    // print stuff to make sure we are parsing correctly
    printf("%s: response descriptor:\n", request_name);
    print_RpLidarResponseDescriptor(4, response_descriptor);
    if (response_descriptor.data_response_length != sizeof(RpLidarScanDataResponse))
        printf("ERROR: %s: response_descriptor.data_response_length != sizeof(RpLidarDeviceInfo)\n", request_name);
    // receive SCAN data response packet, repeat as many times as you want.
    uint32_t scans_to_poll = 4;
    RpLidarScanDataResponse scan_data_response;
    for (uint32_t i = 0; i < scans_to_poll; ++i) {
        while (stream_read_exact(pHndl->rx_hndl, &scan_data_response, response_descriptor.data_response_length, 100) != eRpLidarOK)
            printf("ERROR: %s(%lu): cannot read data response from Serial3\n", request_name, i);
        printf("%s(%lu): data response:\n", request_name, i);
        print_RpLidarScanDataResponse(4, scan_data_response);
    }

    // send a STOP request to stop the scanning
    request_name = "STOP";
    // send a STOP request
    // wait at least 10ms
    request_packet_no_payload.command = COMMAND_STOP;
    request_size = sizeof(request_packet_no_payload);
    memcpy(request, &request_packet_no_payload, request_size);
    // print to make sure it's prepared correctly
    printf("%s: request packet: \n", request_name);
    print_RpLidarRequestNoPayload(4, request_packet_no_payload);
    if (serial_write(pHndl->pSerial, request, request_size, 10) != eSerialOK)
        printf("cannot write STOP request to Serial3\n");
    vTaskDelayUntil(&last_wake_time, 10);
    printf("at least 10 milliseconds have past since sending STOP request to Serial3\n");
    // IMPORTANT: discard any lingering data left in receive buffer.
    while (xStreamBufferReceive(pHndl->rx_hndl, request, sizeof(request), 0) > 0);

    // SCAN
    // this time to Calculate RPLIDAR Scanning Speed
    request_name = "SCAN";
    // prepare SCAN request
    // send SCAN request
    // receive SCAN response descriptor
    // The data response packets related to every measurement sample results will be sent out continuously only after the motor rotation becomes stable.
    // receive SCAN data response packet, repeat as many times as you want.
    // send STOP request to stop scanning
    // discard any SCAN data response packets remaining in the receive buffer

    // prepare request packet
    request_packet_no_payload.command = COMMAND_SCAN;
    // print to make sure it's prepared correctly
    printf("%s: request packet: \n", request_name);
    print_RpLidarRequestNoPayload(4, request_packet_no_payload);
    // send request packet
    if (WRITE(&request_packet_no_payload, sizeof(request_packet_no_payload)) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // print stuff so we know what was sent
    // receive response descriptor
    if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
        printf("ERROR: cannot read response descriptor\n");
    if (!(response_descriptor.start_flag1==START_FLAG1 && response_descriptor.start_flag2==START_FLAG2))
        printf("ERROR: invalid format on RPLidar response\n");
    // print stuff to make sure we are parsing correctly
    printf("%s: response descriptor:\n", request_name);
    print_RpLidarResponseDescriptor(4, response_descriptor);
    if (response_descriptor.data_response_length != sizeof(RpLidarScanDataResponse))
        printf("ERROR: %s: response_descriptor.data_response_length != sizeof(RpLidarDeviceInfo)\n", request_name);
    // receive SCAN data response packet, repeat as many times as you want.
    scans_to_poll = 4;
    for (uint32_t i = 0; i < scans_to_poll; ++i) {
        while (stream_read_exact(pHndl->rx_hndl, &scan_data_response, response_descriptor.data_response_length, 100) != eRpLidarOK)
            printf("ERROR: %s(%lu): cannot read data response from Serial3\n", request_name, i);
        printf("%s(%lu): data response:\n", request_name, i);
        print_RpLidarScanDataResponse(4, scan_data_response);
    }

    // send a STOP request to stop the scanning
    request_name = "STOP";
    // send a STOP request
    // wait at least 10ms
    request_packet_no_payload.command = COMMAND_STOP;
    request_size = sizeof(request_packet_no_payload);
    memcpy(request, &request_packet_no_payload, request_size);
    // print to make sure it's prepared correctly
    printf("%s: request packet: \n", request_name);
    print_RpLidarRequestNoPayload(4, request_packet_no_payload);
    if (serial_write(pHndl->pSerial, request, request_size, 10) != eSerialOK)
        printf("cannot write STOP request to Serial3\n");
    vTaskDelayUntil(&last_wake_time, 10);
    printf("at least 10 milliseconds have past since sending STOP request to Serial3\n");
    // IMPORTANT: discard any lingering data left in receive buffer.
    while (xStreamBufferReceive(pHndl->rx_hndl, request, sizeof(request), 0) > 0);

    // EXPRESS_SCAN
    request_name = "EXPRESS_SCAN";
    // prepare SCAN request
    // send SCAN request
    // receive SCAN response descriptor
    // The data response packets related to every measurement sample results will be sent out continuously only after the motor rotation becomes stable.
    // receive SCAN data response packet, repeat as many times as you want.

    // prepare request packet
    request_packet_with_payload.command = COMMAND_EXPRESS_SCAN;
    request_packet_with_payload.payload_size = 0x05;
    request_packet_with_payload.payload[0] = (uint8_t)typical_scan_mode;
    request_packet_with_payload.payload[1] = 0x00;
    request_packet_with_payload.payload[2] = 0x00;
    request_packet_with_payload.payload[3] = 0x00;
    request_packet_with_payload.payload[4] = 0x00;
    request_packet_with_payload.checksum = checksum_RpLidarRequestPacketWithPayload(&request_packet_with_payload);
    request_size = memcpy_RpLidarRequestPacketWithPayload(request, &request_packet_with_payload);
    // print to make sure it's prepared correctly
    // printf("%s: request packet: \n", request_name);
    // print_RpLidarRequestWithPayload(4, &request_packet_with_payload);
    // send request packet
    if (WRITE(&request, request_size) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // receive response descriptor
    if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
        printf("ERROR: %s: cannot read response descriptor\n", request_name);
    if (!(response_descriptor.start_flag1==START_FLAG1 && response_descriptor.start_flag2==START_FLAG2))
        printf("ERROR: %s: invalid format on RPLidar response descriptor\n", request_name);
    // print stuff to make sure we are parsing correctly
    // printf("%s: response descriptor:\n", request_name);
    // print_RpLidarResponseDescriptor(4, response_descriptor);
    if (response_descriptor.data_response_length != sizeof(RpLidarExpressScanDataResponse))
        printf("ERROR: %s: response_descriptor.data_response_length != expected_data_response_length\n", request_name);
    // receive data response packets, print out two just to see what they look like.
    const uint32_t express_scans_to_poll = 2;
    RpLidarExpressScanDataResponse express_scan_data_response;
    for (uint32_t i = 0; i < express_scans_to_poll; ++i) {
        while (stream_read_exact(pHndl->rx_hndl, &express_scan_data_response, response_descriptor.data_response_length, 100) != eRpLidarOK)
            printf("ERROR: %s(%lu): cannot read data response from Serial3\n", request_name, i);
        printf("%s(%lu): data response:\n", request_name, i);
        print_RpLidarExpressScanDataResponse(4, &express_scan_data_response);
    }
    // send a STOP request to stop the scanning
    request_name = "STOP";
    // send a STOP request
    // wait at least 10ms
    request_packet_no_payload.command = COMMAND_STOP;
    request_size = sizeof(request_packet_no_payload);
    memcpy(request, &request_packet_no_payload, request_size);
    // print to make sure it's prepared correctly
    printf("%s: request packet: \n", request_name);
    print_RpLidarRequestNoPayload(4, request_packet_no_payload);
    if (serial_write(pHndl->pSerial, request, request_size, 10) != eSerialOK)
        printf("cannot write STOP request to Serial3\n");
    vTaskDelayUntil(&last_wake_time, 10);
    printf("at least 10 milliseconds have past since sending STOP request to Serial3\n");
    // IMPORTANT: discard any lingering data left in receive buffer.
    while (xStreamBufferReceive(pHndl->rx_hndl, request, sizeof(request), 0) > 0);
    

    // EXPRESS_SCAN
    // this time to Calculate RPLIDAR Scanning Speed
    request_name = "EXPRESS_SCAN";
    // prepare SCAN request
    // send SCAN request
    // receive SCAN response descriptor
    // The data response packets related to every measurement sample results will be sent out continuously only after the motor rotation becomes stable.
    // receive SCAN data response packet, repeat as many times as you want.

    // prepare request packet
    request_packet_with_payload.command = COMMAND_EXPRESS_SCAN;
    request_packet_with_payload.payload_size = 0x05;
    request_packet_with_payload.payload[0] = (uint8_t)typical_scan_mode;
    request_packet_with_payload.payload[1] = 0x00;
    request_packet_with_payload.payload[2] = 0x00;
    request_packet_with_payload.payload[3] = 0x00;
    request_packet_with_payload.payload[4] = 0x00;
    request_packet_with_payload.checksum = checksum_RpLidarRequestPacketWithPayload(&request_packet_with_payload);
    request_size = memcpy_RpLidarRequestPacketWithPayload(request, &request_packet_with_payload);
    printf("%s: request packet: \n", request_name);
    print_RpLidarRequestWithPayload(4, &request_packet_with_payload);
    uint16_t const rotations = 32;
    struct systime t[rotations];
    uint32_t packets_read = 0;
    for (uint16_t r = 0; r < rotations; ++r) {
        // send request packet
        if (WRITE(&request, request_size) != eSerialOK)
            printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
        // receive response descriptor
        if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
            printf("ERROR: %s: cannot read response descriptor\n", request_name);
        if (!(response_descriptor.start_flag1==START_FLAG1 && response_descriptor.start_flag2==START_FLAG2))
            printf("ERROR: %s: invalid format on RPLidar response descriptor\n", request_name);
        if (response_descriptor.data_response_length != sizeof(RpLidarExpressScanDataResponse))
            printf("ERROR: %s: response_descriptor.data_response_length != expected_data_response_length\n", request_name);
        // receive data response packets.
        do {
            while (stream_read_exact(pHndl->rx_hndl, &express_scan_data_response, response_descriptor.data_response_length, 100) != eRpLidarOK);
            ++packets_read;
        } while (express_scan_data_response.header.start != 1);
        systime_fromTicks(xTaskGetTickCount(), t+r);
        printf("r = %u\n", r);
    }
    // send a STOP request to stop the scanning
    request_name = "STOP";
    // send a STOP request
    // wait at least 10ms
    request_packet_no_payload.command = COMMAND_STOP;
    request_size = sizeof(request_packet_no_payload);
    memcpy(request, &request_packet_no_payload, request_size);
    // print to make sure it's prepared correctly
    printf("%s: request packet: \n", request_name);
    print_RpLidarRequestNoPayload(4, request_packet_no_payload);
    if (serial_write(pHndl->pSerial, request, request_size, 10) != eSerialOK)
        printf("cannot write STOP request to Serial3\n");
    vTaskDelayUntil(&last_wake_time, 10);
    printf("at least 10 milliseconds have past since sending STOP request to Serial3\n");
    // IMPORTANT: discard any lingering data left in receive buffer.
    while (xStreamBufferReceive(pHndl->rx_hndl, request, sizeof(request), 0) > 0);

    // now do the analysis
    // milliseconds
    int duration; 
    // revolutions per minute (rpm)
    double speed;
    printf("rotations = %u; packets_read = %lu\n", rotations, packets_read);
    for (uint16_t i = 0; i+1 < rotations; ++i) {
        // print_RpLidarExpressScanDataResponseHeader(0, headers+i);
        // duration = (t[i].secs*1000 + t[i].msecs) - (t[i].secs*1000 + t[i].msecs);
        duration = (t[i+1].secs - t[i].secs)*1000 + t[i+1].msecs - t[i].msecs;
        speed = 60.0 / ((double) duration) * 1000.0;
        printf("%u-%u: DeltaT = %d(ms); speed = %lf(rpm)\n", i, i+1, duration, speed);
    }
    // print_RpLidarExpressScanDataResponseHeader(0, headers+rotations-1);
#endif

#if 0
    // struct udev_pkt_lidar sequence number angle ranges:
    printf("struct udev_pkt_lidar sequence number angle ranges:\n");
    for (int seq = 0; seq < UDEV_SEQ_MAX; ++seq) {
        printf("    %d: [%f, %f)\n", seq, udev_lidar_angle(seq, 0), udev_lidar_angle(seq, UDEV_LIDAR_POINTS));
    }
#endif 

// #if 0
    // Now to do express scans, with sliding window, 
    // convert them into `struct udev_pkt_lidar`
    request_name = "EXPRESS_SCAN";
    request_packet_with_payload.command = COMMAND_EXPRESS_SCAN;
    request_packet_with_payload.payload_size = 0x05;
    request_packet_with_payload.payload[0] = (uint8_t)typical_scan_mode;
    request_packet_with_payload.payload[1] = 0x00;
    request_packet_with_payload.payload[2] = 0x00;
    request_packet_with_payload.payload[3] = 0x00;
    request_packet_with_payload.payload[4] = 0x00;
    request_packet_with_payload.checksum = checksum_RpLidarRequestPacketWithPayload(&request_packet_with_payload);
    request_size = memcpy_RpLidarRequestPacketWithPayload(request, &request_packet_with_payload);
    printf("%s: request packet: \n", request_name);
    print_RpLidarRequestWithPayload(4, &request_packet_with_payload);
    // send the request packet
    while (serial_write(pHndl->pSerial, request, request_size, 10) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // get response descriptor
    while (stream_read_exact(pHndl->rx_hndl, &response_descriptor, sizeof(response_descriptor), 10) != eRpLidarOK)
        printf("ERROR: %s: cannot read response descriptor\n", request_name);
    printf("%s: response descriptor:\n", request_name);
    print_RpLidarResponseDescriptor(4, response_descriptor);
    // validate response descriptor
    if (!(response_descriptor.start_flag1==START_FLAG1 && response_descriptor.start_flag2==START_FLAG2))
        printf("ERROR: %s: invalid format on RPLidar response descriptor\n", request_name);
    if (response_descriptor.data_response_length != sizeof(RpLidarExpressScanDataResponse))
        printf("ERROR: %s: response_descriptor.data_response_length != expected_data_response_length\n", request_name);

    // receive data responses then convert to usb packets
    #define PI 3.1415926535f
    #define WINDOW_SIZE 2
    RpLidarExpressScanDataResponseHeader header[WINDOW_SIZE];
    uint16_t angle_diff_q6[WINDOW_SIZE];
    // have[i] = points in express scan cabin for header[i] remaining in stream buffer.
    uint8_t have[WINDOW_SIZE];
    // angles in radians, because udev_lidar_index takes angle in radians
    struct udev_pkt_lidar usb_pkt[WINDOW_SIZE];
    // set usb_pkt ids now because they don't change.
    usb_pkt[0].hdr.id = pHndl->id;
    usb_pkt[1].hdr.id = pHndl->id;
    float angle_diff[WINDOW_SIZE], start_angle[WINDOW_SIZE], end_angle[WINDOW_SIZE];
    int seq, iter;
    // want[i] = points that the usb_pkt[i] wants to take to be full.
    uint8_t want[WINDOW_SIZE];
    // take[i][j] = number of distances usb_pkt[i] to take from express scan cabin with header[j].
    uint8_t take[WINDOW_SIZE][WINDOW_SIZE];
    
    // index for header
    uint8_t h = 0;
    // Hh
    // read header h
    while (stream_read_exact(pHndl->rx_hndl, header+h, sizeof(*header), 100) != eRpLidarOK)
        printf("ERROR: %s: cannot read header from Serial3\n", request_name);
    have[h] = RPLIDAR_EXPRESS_SCAN_CABIN_SIZE;
    printf("header[%u]:\n", h);
    print_RpLidarExpressScanDataResponseHeader(4, header+h);

    // for (uint8_t i = 0; i < 2; ++i, h = 1 - h){
    for (;; h = 1 - h){
        // take Hh distances into usb_pkt[0]
        want[0] = UDEV_LIDAR_POINTS;
        take[0][h] = 0, take[0][1-h] = 0;
        take[0][h] = min(want[0], have[h]);
        while (stream_read_exact(pHndl->rx_hndl, usb_pkt[0].distances + take[0][1-h], take[0][h]*sizeof(*usb_pkt[0].distances), 100) != eRpLidarOK)
            printf("ERROR: %s: cannot read cabin from Serial3\n", request_name);
        want[0] -= take[0][h], have[h] -= take[0][h];
        assert(want[0] == 0);
        // can't send usb_pkt[0] yet because we don't yet know the sequence number -> read another packet to get the sequence number.
        want[1] = UDEV_LIDAR_POINTS;
        take[1][h] = 0, take[1][1-h] = 0;
        take[1][h] = min(want[1], have[h]);
        // read remaining Hh distances into usb_pkt[1]
        while (stream_read_exact(pHndl->rx_hndl, usb_pkt[1].distances + take[1][1-h], take[1][h]*sizeof(*usb_pkt[1].distances), 100) != eRpLidarOK)
            printf("ERROR: %s: cannot read cabin from Serial3\n", request_name);
        want[1] -= take[1][h], have[h] -= take[1][h];
        assert(have[h] == 0);
    
        // H1-h
        // read next header to get the next start angle
        while (stream_read_exact(pHndl->rx_hndl, header+1-h, sizeof(*header), 100) != eRpLidarOK)
            printf("ERROR: %s: cannot read header from Serial3\n", request_name);
        have[1-h] = RPLIDAR_EXPRESS_SCAN_CABIN_SIZE;
        // printf("header[%u]:\n", 1-h);
        // print_RpLidarExpressScanDataResponseHeader(4, header+1-h);
        // calculate start_angle and end_angle for usb_pkt[0], from header[h]->header[1-h].
        angle_diff_q6[h] = (header[1-h].start_angle_q6 < header[h].start_angle_q6)*(360U*64U) + header[1-h].start_angle_q6 - header[h].start_angle_q6;
        angle_diff[h] = angle_diff_q6[h]/64.0f*PI/180.0f;
        start_angle[0] = header[h].start_angle_q6/64.0f*PI/180.0f + angle_diff[h]/40.0f;
        end_angle[0] = header[h].start_angle_q6/64.0f*PI/180.0f + angle_diff[h]*take[0][h]/40.0f;
        start_angle[0] -= (start_angle[0] >= 2*PI)*2*PI;
        end_angle[0] -= (end_angle[0] >= 2*PI)*2*PI;
        // calculate start_angle for usb_pkt[1]. can't do the end_angle yet because we don't have the next header
        start_angle[1] = end_angle[0] + angle_diff[h]/40.0f;
        start_angle[1] -= (start_angle[1] >= 2*PI)*2*PI;
        // calculate sequence usb_pkt[0].hdr.sequence
        udev_lidar_index(start_angle[0], &seq, &iter);
        usb_pkt[0].hdr.sequence = seq;
        // printf("start_angle[0]=%f sequence=%d\n", start_angle[0], seq);
        // udev_lidar_index(end_angle[0], &seq, &iter);
        // printf("end_angle[0]=%f sequence=%d\n", end_angle[0], seq);
        // set usb_pkt[0].hdr.len to be full because we are filling it up. However, ideally we would want to cut the length short to make the start_angle[0] and end_angle[0] fit inside the same sequence number.
        usb_pkt[0].hdr.len = UDEV_LIDAR_POINTS;
        // don't know what this is supposed to be, TODO: find out.
        usb_pkt[0].distance_sum = 0;
        // output usb_pkt[0]. done with usb_pkt[0], it can now be overwritten.
        printf("usb_pkt[0]\n");
        while (xQueueSendToBack(tx, usb_pkt+0, 100) != pdTRUE);
        // print_udev_pkt_lidar(4, usb_pkt+0);
        // fill up the rest of usb_pkt[1] with distances from H1
        take[1][1-h] = min(want[1], have[1-h]);
        while (stream_read_exact(pHndl->rx_hndl, usb_pkt[1].distances + take[1][h], take[1][1-h]*sizeof(*usb_pkt[1].distances), 100) != eRpLidarOK)
            printf("ERROR: %s: cannot read cabin from Serial3\n", request_name);
        want[1] -= take[1][1-h], have[1-h] -= take[1][1-h];
        assert(want[1] == 0);
        // can't send usb_pkt[1] yet because we don't yet know the sequence number -> read another header to get the sequence number.
        // read remaining H1-h distances into usb_pkt[0]
        want[0] = UDEV_LIDAR_POINTS;
        take[0][h] = 0, take[0][1-h] = 0;
        take[0][1-h] = min(want[0], have[1-h]);
        while (stream_read_exact(pHndl->rx_hndl, usb_pkt[0].distances + take[0][h], take[0][1-h]*sizeof(*usb_pkt[0].distances), 100) != eRpLidarOK)
            printf("ERROR: %s: cannot read cabin from Serial3\n", request_name);
        want[0] -= take[0][1-h], have[1-h] -= take[0][1-h];
        assert(have[1-h] == 0);
    
        // Hh
        // read next header to get the start_angle
        while (stream_read_exact(pHndl->rx_hndl, header+h, sizeof(*header), 100) != eRpLidarOK)
            printf("ERROR: %s: cannot read header from Serial3\n", request_name);
        have[h] = RPLIDAR_EXPRESS_SCAN_CABIN_SIZE;
        // printf("header[%u]:\n", h);
        // print_RpLidarExpressScanDataResponseHeader(4, header+h);
        // calculate end_angle for usb_pkt[1], from header[1-h]->header[h]
        angle_diff_q6[1-h] = (header[h].start_angle_q6 < header[1-h].start_angle_q6)*(360U*64U) + header[h].start_angle_q6 - header[1-h].start_angle_q6;
        angle_diff[1-h] = angle_diff_q6[1-h]/64.0f*PI/180.0f;
        end_angle[1] = header[1-h].start_angle_q6/64.0f*PI/180.0f + angle_diff[1-h]*take[1][1-h]/40.0f;
        end_angle[1] -= (end_angle[1] >= 2*PI)*2*PI;
        // calculate start_angle for usb_pkt[0]. can't do the end_angle yet because we dont know the next header
        start_angle[0] = end_angle[1] + angle_diff[1-h]/40.0f;
        start_angle[1] -= (start_angle[1] >= 2*PI)*2*PI;
        // calculate sequence usb_pkt[1].hdr.sequence
        udev_lidar_index(start_angle[1], &seq, &iter);
        usb_pkt[1].hdr.sequence = seq;
        // printf("start_angle[1]=%f sequence=%d\n", start_angle[1], seq);
        // udev_lidar_index(end_angle[1], &seq, &iter);
        // printf("end_angle[1]=%f sequence=%d\n", end_angle[1], seq);
        // set usb_pkt[1].hdr.len to be full because we are filling it up. However, ideally we would want to cut the length short to make the start_angle[1] and end_angle[1] fit inside the same sequence number.
        usb_pkt[1].hdr.len = UDEV_LIDAR_POINTS;
        // don't know what this is supposed to be, TODO: figure it out later.
        usb_pkt[1].distance_sum = 0;
        // output usb_pkt[1]. done with usb_pkt[1], it can now be overwritten.
        printf("usb_pkt[1]\n");
        while (xQueueSendToBack(tx, usb_pkt+1, 100) != pdTRUE);
        // print_udev_pkt_lidar(4, usb_pkt+1);
        // fill up the rest of usb_pkt[0] with Hh distances
        take[0][h] = min(want[0], have[h]);
        while (stream_read_exact(pHndl->rx_hndl, usb_pkt[1].distances + take[1][1-h], take[0][h]*sizeof(*usb_pkt[1].distances), 100) != eRpLidarOK)
            printf("ERROR: %s: cannot read cabin from Serial3\n", request_name);
        want[0] -= take[0][h], have[h] -= take[0][h];
        assert(want[0] == 0);
        // can't send usb_pkt[0] yet because we don't yet know the sequence number -> read another header to get the sequence number.
        // read remaining Hh distances into usb_pkt[1]
        want[1] = UDEV_LIDAR_POINTS;
        take[1][h] = 0, take[1][1-h] = 0;
        take[1][h] = min(want[1], have[h]);
        while (stream_read_exact(pHndl->rx_hndl, usb_pkt[1].distances + take[1][1-h], take[1][h]*sizeof(*usb_pkt[1].distances), 100) != eRpLidarOK)
            printf("ERROR: %s: cannot read cabin from Serial3\n", request_name);
        want[1] -= take[1][h], have[h] -= take[1][h];
        assert(want[0] == 0);
        assert(have[h] == 0);

        // H1-h
        // read next header to get the next start angle
        while (stream_read_exact(pHndl->rx_hndl, header+1-h, sizeof(*header), 100) != eRpLidarOK)
            printf("ERROR: %s: cannot read header from Serial3\n", request_name);
        have[1-h] = RPLIDAR_EXPRESS_SCAN_CABIN_SIZE;
        // printf("header[%u]:\n", 1-h);
        // print_RpLidarExpressScanDataResponseHeader(4, header+1-h);
        // calculate end_angle for usb_pkt[0], from header[h] -> header[1-h]
        angle_diff_q6[h] = (header[1-h].start_angle_q6 < header[h].start_angle_q6)*(360U*64U) + header[1-h].start_angle_q6 - header[h].start_angle_q6;
        angle_diff[h] = angle_diff_q6[h]/64.0f*PI/180.0f;
        end_angle[0] = header[h].start_angle_q6/64.0f*PI/180.0f + angle_diff[0]*take[0][h]/40.0f;
        end_angle[0] -= (end_angle[0] >= 2*PI)*2*PI;
        // calculate start_angle for usb_pkt[1]
        start_angle[1] = end_angle[0] + angle_diff[h]/40.0f;
        start_angle[1] -= (start_angle[1] >= 2*PI)*2*PI;
        // calculate sequence usb_pkt[0].hdr.sequence
        udev_lidar_index(start_angle[0], &seq, &iter);
        usb_pkt[0].hdr.sequence = seq;
        // printf("start_angle[0]=%f sequence=%d\n", start_angle[0], seq);
        // udev_lidar_index(end_angle[0], &seq, &iter);
        // printf("end_angle[0]=%f sequence=%d\n", end_angle[0], seq);
        // set usb_pkt[0].hdr.len to be full because we are filling it up. However, maybe we would want to cut the length short to make the start_angle[0] and end_angle[0] fit inside the same sequence number.
        usb_pkt[0].hdr.len = UDEV_LIDAR_POINTS;
        // don't know what this is supposed to be, TODO: find out.
        usb_pkt[0].distance_sum = 0;
        // output usb_pkt[0]. done with usb_pkt[0], it can now be overwritten.
        printf("usb_pkt[0]\n");
        while (xQueueSendToBack(tx, usb_pkt+0, 100) != pdTRUE);
        // print_udev_pkt_lidar(4, usb_pkt+0);
        // we can also calculate the end_angle for usb_pkt[1] since we have its next header
        end_angle[1] = header[1-h].start_angle_q6/64.0f*PI/180.0f;
        end_angle[1] -= (end_angle[1] >= 2*PI)*2*PI;
        // calculate sequence usb_pkt[1].hdr.sequence
        udev_lidar_index(start_angle[1], &seq, &iter);
        usb_pkt[1].hdr.sequence = seq;
        // printf("start_angle[1]=%f sequence=%d\n", start_angle[1], seq);
        // udev_lidar_index(end_angle[1], &seq, &iter);
        // printf("end_angle[1]=%f sequence=%d\n", end_angle[1], seq);
        // set usb_pkt[1].hdr.len to be full because we are filling it up. However, ideally we would want to cut the length short to make the start_angle[1] and end_angle[1] fit inside the same sequence number.
        usb_pkt[1].hdr.len = UDEV_LIDAR_POINTS;
        // don't know what this is supposed to be, TODO: figure it out later.
        usb_pkt[1].distance_sum = 0;
        // output usb_pkt[1]. done with usb_pkt[1], it can now be overwritten.
        printf("usb_pkt[1]\n");
        while (xQueueSendToBack(tx, usb_pkt+1, 100) != pdTRUE);
        // print_udev_pkt_lidar(4, usb_pkt+1);
    }

    // send a STOP request to stop the scanning
    request_name = "STOP";
    // send a STOP request
    // wait at least 10ms
    request_packet_no_payload.command = COMMAND_STOP;
    request_size = sizeof(request_packet_no_payload);
    memcpy(request, &request_packet_no_payload, request_size);
    // print to make sure it's prepared correctly
    printf("%s: request packet: \n", request_name);
    print_RpLidarRequestNoPayload(4, request_packet_no_payload);
    if (serial_write(pHndl->pSerial, request, request_size, 10) != eSerialOK)
        printf("cannot write STOP request to Serial3\n");
    vTaskDelayUntil(&last_wake_time, 10);
    printf("at least 10 milliseconds have past since sending STOP request to Serial3\n");
    // IMPORTANT: discard any lingering data left in receive buffer.
    while (xStreamBufferReceive(pHndl->rx_hndl, request, sizeof(request), 0) > 0);
// #endif

    for (;;) {
        printf("vRpLidar_tsk\n");
        vTaskDelayUntil(&last_wake_time, 1000);
    }
}
