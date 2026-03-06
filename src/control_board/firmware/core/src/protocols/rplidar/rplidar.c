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

eRpLidarError rplidar_notify(RpLidar_t* pHndl, TaskHandle_t* const pNotify_tskHndl) {
    if (!pHndl)
        return eRpLidarNULL;
    if (!pNotify_tskHndl)
        return eRpLidarNULL;
#warning "rplidar_notify not implimented"
    return eRpLidarOK;
}

eRpLidarError rplidar_read(RpLidar_t* pHndl, RpLidarScanArray* const pScan) {
    (void)pHndl;
    (void)pScan;
#warning "rplidar_read not implimented"
    // 1. Aquire read lock
    // 2. Memcpy to the dest
    // 3. Release read lock
    return eRpLidarOK;
}

static void print_bytes(const void *data, size_t len) {
    const uint8_t *p = data;

    for (size_t i = 0; i < len; i++) {
        printf("byte %u: %02X\n", i, p[i]);
    }
}

static void print_RpLidarRequestNoPayload(uint16_t indent, RpLidarRequestNoPayload rnp) {
    printf("%*sstart_flag: 0x%02X\n", indent, "", rnp.start_flag);
    printf("%*scommand: 0x%02X\n", indent, "", rnp.command);
}

static void print_RpLidarRequestWithPayload(uint16_t indent, RpLidarRequestWithPayload *rwp) {
    printf("%*sstart_flag: 0x%02X\n", indent, "", rwp->start_flag);
    printf("%*scommand: 0x%02X\n", indent, "", rwp->command);
    printf("%*spayload_size: 0x%02X\n", indent, "", rwp->payload_size);
    printf("%*spayload:", indent, "");
    uint8_t i = 0, *payload = rwp->payload;
    do {
        printf(" %02X", payload[i]);
    } while (++i < rwp->payload_size);
    printf("\n");
    printf("%*schecksum: 0x%02X\n", indent, "", rwp->checksum);
}

static void print_RpLidarResponseDescriptor(uint16_t indent, RpLidarResponseDescriptor rd) {
    printf("%*sstart_flag1: 0x%02X\n", indent, "", rd.start_flag1);
    printf("%*sstart_flag2: 0x%02X\n", indent, "", rd.start_flag2);
    printf("%*sdata_response_length: %u\n", indent, "", rd.data_response_length);
    printf("%*ssend_mode: 0x%1X\n", indent, "", rd.send_mode); 
    printf("%*sdata_type: 0x%02X\n", indent, "", rd.data_type);  
}

static void print_RpLidarDeviceInfo(uint16_t indent, RpLidarDeviceInfo device_info) {
    printf("%*smajor_model: %X\n", indent, "", device_info.major_model);
    printf("%*ssub_model: %X\n", indent, "", device_info.sub_model);
    printf("%*sfirmware_minor: %X\n", indent, "", device_info.firmware_minor);
    printf("%*sfirmware_major: %X\n", indent, "", device_info.firmware_major);
    printf("%*shardware: %X\n", indent, "", device_info.hardware);
    printf("%*sserialnumber: %X-%X-%X-%X-%X-%X-%X-%X-%X-%X-%X-%X-%X-%X-%X-%X\n", 
         indent, "",
         device_info.serialnumber[0], 
         device_info.serialnumber[1], 
         device_info.serialnumber[2],
         device_info.serialnumber[3],
         device_info.serialnumber[4],
         device_info.serialnumber[5],
         device_info.serialnumber[6],
         device_info.serialnumber[7],
         device_info.serialnumber[8],
         device_info.serialnumber[9],
         device_info.serialnumber[10],
         device_info.serialnumber[11],
         device_info.serialnumber[12],
         device_info.serialnumber[13],
         device_info.serialnumber[14],
         device_info.serialnumber[15]
        );
}

static void print_RpLidarDeviceHealth(uint16_t indent, RpLidarDeviceHealth device_health) {
    printf("%*sstatus: %u\n", indent, "", device_health.status);
    printf("%*serror_code: %u\n", indent, "", device_health.error_code);
}

static void print_RpLidarSampleRate(uint16_t indent, RpLidarSampleRate sample_rate) {
    printf("%*sTstandard: %u\n", indent, "", sample_rate.Tstandard);
    printf("%*sTexpress: %u\n", indent, "", sample_rate.Texpress);
}

static void print_RpLidarConfResponseDataWithU8Payload(const uint16_t indent, const RpLidarConfResponseDataWithU8Payload data, const char *const payload_name) {
    printf("%*stype: 0x%02lX\n", indent, "", data.type);
    printf("%*s%s: 0x%02X\n", indent, "", payload_name, data.payload);
}

static void print_RpLidarConfResponseDataWithU16Payload(const uint16_t indent, const RpLidarConfResponseDataWithU16Payload data, const char *const payload_name) {
    printf("%*stype: 0x%02lX\n", indent, "", data.type);
    printf("%*s%s: %u\n", indent, "", payload_name, data.payload);
}

static void print_RpLidarConfResponseDataWithU32Payload(const uint16_t indent, const RpLidarConfResponseDataWithU32Payload data, const char *const payload_name) {
    printf("%*stype: 0x%02lX\n", indent, "", data.type);
    printf("%*s%s: %lu\n", indent, "", payload_name, data.payload);
}

static void print_RpLidarConfResponseDataWithStringPayload(const uint16_t indent, const RpLidarConfResponseDataWithStringPayload data, const char *const payload_name) {
    printf("%*stype: 0x%02lX\n", indent, "", data.type);
    printf("%*s%s: %s\n", indent, "", payload_name, data.payload);
}

static void print_RpLidarScanDataResponse(const uint16_t indent, const RpLidarScanDataResponse scan) {
    printf("%*sstart: %u\n", indent, "", scan.start);
    printf("%*sn_start: %u\n", indent, "", scan.n_start);
    printf("%*squality: %u\n", indent, "", scan.quality);
    printf("%*scheck: %u\n", indent, "", scan.check);
    printf("%*sangle_q6: %d\n", indent, "", scan.angle_q6);
    printf("%*sdistance_q2: %d\n", indent, "", scan.distance_q2);
}

static void print_RpLidarExpressScanDataResponseRaw(const uint16_t indent, const RpLidarExpressScanDataResponseRaw scan) {
    printf("%*schecksum1: %01X\n", indent, "", scan.checksum1);
    printf("%*ssync1: %01X\n", indent, "", scan.sync1);
    printf("%*schecksum2: %01X\n", indent, "", scan.checksum2);
    printf("%*ssync2: %01X\n", indent, "", scan.sync2);
    printf("%*sstart_angle_q6: %u\n", indent, "", scan.start_angle_q6);
    printf("%*sstart: %u\n", indent, "", scan.start);
    printf("%*scabin: %u\n", indent, "", scan.cabin[0]);
    for (uint8_t i = 1; i < sizeof(scan.cabin); ++i)
        printf("%*s       %u\n", indent, "", scan.cabin[i]);
}

static uint8_t checksum_RpLidarRequestPacketWithPayload(RpLidarRequestWithPayload *request_packet) {
    uint8_t checksum = 0 ^ (request_packet->start_flag) ^ (request_packet->command) ^ (request_packet->payload_size);
    uint8_t *payload = request_packet->payload, i = 0, payload_size = request_packet->payload_size;
    do {
        checksum ^= payload[i];
    } while (++i < payload_size);
    return checksum;
}

static uint16_t memcpy_RpLidarRequestPacketWithPayload(char *dst, const RpLidarRequestWithPayload *pkt) {
    memcpy(dst, pkt, offsetof(typeof(*pkt), payload));
    memcpy(dst+offsetof(typeof(*pkt), payload), pkt->payload, pkt->payload_size);
    memcpy(dst+offsetof(typeof(*pkt), payload) + pkt->payload_size, &(pkt->checksum), sizeof(pkt->checksum));
    return sizeof(*pkt) - sizeof(pkt->payload) + (pkt->payload_size);
}


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

void vRpLidar_tsk(void* pvParams){
    RpLidar_t* pHndl = pvParams;
    if (!pHndl) {
        vTaskSuspend(NULL);
    }
    TickType_t last_wake_time = xTaskGetTickCount();
    const char *request_name;
    RpLidarRequestNoPayload request_packet_no_payload = {.start_flag=START_FLAG};
    RpLidarRequestWithPayload request_packet_with_payload;
    RpLidarResponseDescriptor response_descriptor;
    char request[32];
    uint16_t request_size;

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
    printf("%s: request packet: \n", request_name);
    print_RpLidarRequestNoPayload(4, request_packet_no_payload);
    // send request
    // printf("writing %u bytes to Serial3:\n", sizeof(request_packet_no_payload));
    if (WRITE(&request_packet_no_payload, sizeof(request_packet_no_payload)) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // print stuff so we know what was sent
    // print_bytes(&request_packet_no_payload, sizeof(request_packet_no_payload));
    // printf("%s: wrote request packet to Serial3\n", request_name);
    // receive response descriptor
    // printf("reading %u bytes from Serial3:\n", sizeof(response_descriptor));
    if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
        printf("ERROR: cannot read GET_INFO response descriptor\n");
    if (!(response_descriptor.start_flag1==START_FLAG1 && response_descriptor.start_flag2==START_FLAG2))
        printf("ERROR: invalid format on RPLidar response\n");
    // print stuff to make sure we are parsing correctly
    // print_bytes(&response_descriptor, sizeof(response_descriptor));
    // printf("%s: received response descriptor\n", request_name);
    printf("%s: response descriptor:\n", request_name);
    print_RpLidarResponseDescriptor(4, response_descriptor);
    if (response_descriptor.data_response_length != sizeof(RpLidarDeviceInfo))
        printf("ERROR: %s: response_descriptor.data_response_length != sizeof(RpLidarDeviceInfo)\n", request_name);
    // receive data response
    RpLidarDeviceInfo device_info;
    printf("%s: reading %u bytes from Serial3:\n", request_name, sizeof(device_info));
    if (READ(&device_info, response_descriptor.data_response_length) != eRpLidarOK)
        printf("ERROR: %s: cannot read data response\n", request_name);
    // print stuff to make sure we are parsing correctly
    // print_bytes(&device_info, sizeof(device_info));
    // printf("%s: received data response\n", request_name);
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
    printf("%s: request packet:\n", request_name);
    print_RpLidarRequestNoPayload(4, request_packet_no_payload);
    // send a GET_HEALTH request
    if (WRITE(&request_packet_no_payload, sizeof(request_packet_no_payload)) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // receive GET_HEALTH response descriptor
    if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
        printf("ERROR: %s: cannot read response descriptor from Serial3\n", request_name);
    if (!(response_descriptor.start_flag1==START_FLAG1 && response_descriptor.start_flag2==START_FLAG2))
        printf("ERROR: %s: invalid format on RPLidar response\n", request_name);
    // print stuff to make sure we are parsing correctly
    printf("%s: response descriptor:\n", request_name);
    print_RpLidarResponseDescriptor(4, response_descriptor);
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
    printf("%s: request packet:\n", request_name);
    print_RpLidarRequestNoPayload(4, request_packet_no_payload);
    // send request
    if (WRITE(&request_packet_no_payload, sizeof(request_packet_no_payload)) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // receive response descriptor
    if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
        printf("ERROR: %s: cannot read response descriptor from Serial3\n", request_name);
    if (!(response_descriptor.start_flag1==START_FLAG1 && response_descriptor.start_flag2==START_FLAG2))
        printf("ERROR: %s: invalid format on RPLidar response\n", request_name);
    // print stuff to make sure we are parsing correctly
    printf("%s: response descriptor:\n", request_name);
    print_RpLidarResponseDescriptor(4, response_descriptor);
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
    printf("%s: request packet:\n", request_name);
    print_RpLidarRequestWithPayload(4, &request_packet_with_payload);
    // send request packet
    // printf("writing %u bytes to Serial3:\n", request_size);
    if (WRITE(request, request_size) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // print_bytes(request, request_size);
    // printf("%s: wrote request packet to Serial3\n", request_name);
    // receive response descriptor
    // printf("reading %u bytes from Serial3:\n", sizeof(RpLidarResponseDescriptor));
    while (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
        printf("ERROR: %s: cannot read response descriptor from Serial3\n", request_name);
    // print stuff to make sure we are parsing correctly
    // print_bytes(&response_descriptor, sizeof(response_descriptor));
    printf("%s: response descriptor:\n", request_name);
    print_RpLidarResponseDescriptor(4, response_descriptor);
    if (response_descriptor.data_response_length != sizeof(RpLidarConfResponseDataWithU16Payload))
        printf("ERROR: %s: response_descriptor.data_response_length != expected_data_response_length\n", request_name);
    // receive data response
    RpLidarConfResponseDataWithU16Payload scan_mode_count_data_response;
    // printf("reading %u bytes from Serial3:\n", response_descriptor.data_response_length);
    if (READ(&scan_mode_count_data_response, response_descriptor.data_response_length) != eRpLidarOK)
        printf("ERROR: %s: cannot read data response from Serial3\n", request_name);
    // print stuff to make sure we are parsing correctly
    // print_bytes(&response_descriptor, sizeof(scan_mode_count_data_response));
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
        printf("%s(%u): request packet:\n", request_name, mode);
        print_RpLidarRequestWithPayload(4, &request_packet_with_payload);
        // send request packet
        if (WRITE(request, request_size) != eSerialOK)
            printf("ERROR: %s(%u): cannot write request packet to Serial3\n", request_name, mode);
        // receive response descriptor
        if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
            printf("ERROR: %s(%u): cannot read response descriptor from Serial3\n", request_name, mode);
        if (response_descriptor.data_response_length != sizeof(RpLidarConfResponseDataWithU32Payload))
            printf("ERROR: %s(%u): response_descriptor.data_response_length != expected_data_response_length\n", request_name, mode);
        // print stuff to make sure we are parsing correctly
        printf("%s(%u): response descriptor:\n", request_name, mode);
        print_RpLidarResponseDescriptor(4, response_descriptor);
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
        printf("%s(%u): request packet:\n", request_name, mode);
        print_RpLidarRequestWithPayload(4, &request_packet_with_payload);
        // send request packet
        if (WRITE(request, request_size) != eSerialOK)
            printf("ERROR: %s(%u): cannot write request packet to Serial3\n", request_name, mode);
        // receive response descriptor
        if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
            printf("ERROR: %s(%u): cannot read response descriptor from Serial3\n", request_name, mode);
        if (response_descriptor.data_response_length != sizeof(RpLidarConfResponseDataWithU32Payload))
            printf("ERROR: %s(%u): response_descriptor.data_response_length != expected_data_response_length\n", request_name, mode);
        // print stuff to make sure we are parsing correctly
        printf("%s(%u): response descriptor:\n", request_name, mode);
        print_RpLidarResponseDescriptor(4, response_descriptor);
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
        printf("%s(%u): request packet:\n", request_name, mode);
        print_RpLidarRequestWithPayload(4, &request_packet_with_payload);
        // send request packet
        if (WRITE(request, request_size) != eSerialOK)
            printf("ERROR: %s(%u): cannot write request packet to Serial3\n", request_name, mode);
        // receive response descriptor
        if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
            printf("ERROR: %s(%u): cannot read response descriptor from Serial3\n", request_name, mode);
        if (response_descriptor.data_response_length != sizeof(RpLidarConfResponseDataWithU8Payload))
            printf("ERROR: %s(%u): response_descriptor.data_response_length != expected_data_response_length\n", request_name, mode);
        // print stuff to make sure we are parsing correctly
        printf("%s(%u): response descriptor:\n", request_name, mode);
        print_RpLidarResponseDescriptor(4, response_descriptor);
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
    printf("%s: request packet:\n", request_name);
    print_RpLidarRequestWithPayload(4, &request_packet_with_payload);
    // send request packet
    if (WRITE(request, request_size) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // receive response descriptor
    if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
        printf("ERROR: %s: cannot read response descriptor from Serial3\n", request_name);
    // print stuff to make sure we are parsing correctly
    printf("%s: response descriptor:\n", request_name);
    print_RpLidarResponseDescriptor(4, response_descriptor);
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
        printf("%s(%u): request packet:\n", request_name, mode);
        print_RpLidarRequestWithPayload(4, &request_packet_with_payload);
        // send request packet
        if (WRITE(request, request_size) != eSerialOK)
            printf("ERROR: %s(%u): cannot write request packet to Serial3\n", request_name, mode);
        // receive response descriptor
        if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
            printf("ERROR: %s(%u): cannot read response descriptor from Serial3\n", request_name, mode);
        if (response_descriptor.data_response_length > sizeof(RpLidarConfResponseDataWithStringPayload))
            printf("ERROR: %s(%u): response_descriptor.data_response_length > maximum_data_response_length\n", request_name, mode);
        // print stuff to make sure we are parsing correctly
        printf("%s(%u): response descriptor:\n", request_name, mode);
        print_RpLidarResponseDescriptor(4, response_descriptor);
        // receive data response
        RpLidarConfResponseDataWithStringPayload scan_mode_name_data_response;
        if (READ(&scan_mode_name_data_response, response_descriptor.data_response_length) != eRpLidarOK)
            printf("ERROR: %s(%u): cannot read data response from Serial3\n", request_name, mode);
        // print stuff to make sure we are parsing correctly
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
    // printf("writing %u bytes to Serial3:\n", sizeof(request_packet_no_payload));
    if (WRITE(&request_packet_no_payload, sizeof(request_packet_no_payload)) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // print stuff so we know what was sent
    // print_bytes(&request_packet_no_payload, sizeof(request_packet_no_payload));
    // printf("%s: wrote request packet to Serial3\n", request_name);
    // receive response descriptor
    // printf("reading %u bytes from Serial3:\n", sizeof(response_descriptor));
    if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
        printf("ERROR: cannot read response descriptor\n");
    if (!(response_descriptor.start_flag1==START_FLAG1 && response_descriptor.start_flag2==START_FLAG2))
        printf("ERROR: invalid format on RPLidar response\n");
    // print stuff to make sure we are parsing correctly
    // print_bytes(&response_descriptor, sizeof(response_descriptor));
    // printf("%s: received response descriptor\n", request_name);
    printf("%s: response descriptor:\n", request_name);
    print_RpLidarResponseDescriptor(4, response_descriptor);
    if (response_descriptor.data_response_length != sizeof(RpLidarScanDataResponse))
        printf("ERROR: %s: response_descriptor.data_response_length != sizeof(RpLidarDeviceInfo)\n", request_name);
    // wait for motor speed to pick up
    // vTaskDelayUntil(&last_wake_time, 2000);
    // receive SCAN data response packet, repeat as many times as you want.
    uint32_t scans_to_poll = 4;
    RpLidarScanDataResponse scan_data_response;
    for (uint32_t i = 0; i < scans_to_poll; ++i) {
        // printf("%s(%lu): reading %u bytes from Serial3:\n", request_name, i, response_descriptor.data_response_length);
        while (stream_read_exact(pHndl->rx_hndl, &scan_data_response, response_descriptor.data_response_length, 100) != eRpLidarOK)
            printf("ERROR: %s(%lu): cannot read data response from Serial3\n", request_name, i);
        // print_bytes(&scan_data_response, sizeof(scan_data_response));
        // printf("%s(%lu): received data response\n", request_name, i);
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


    // EXPRESS_
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
    memcpy(request, &request_packet_with_payload, offsetof(typeof(request_packet_with_payload), payload));
    memcpy(request+offsetof(typeof(request_packet_with_payload), payload), request_packet_with_payload.payload, request_packet_with_payload.payload_size);
    memcpy(request+offsetof(typeof(request_packet_with_payload), payload) + request_packet_with_payload.payload_size, &request_packet_with_payload.checksum, sizeof(request_packet_with_payload.checksum));
    request_size = sizeof(request_packet_with_payload) - sizeof(request_packet_with_payload.payload) + request_packet_with_payload.payload_size;
    // print to make sure it's prepared correctly
    printf("%s: request packet: \n", request_name);
    print_RpLidarRequestWithPayload(4, &request_packet_with_payload);
    // send request packet
    // printf("writing %u bytes to Serial3:\n", request_size);
    if (WRITE(&request, request_size) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // print stuff so we know what was sent
    // print_bytes(&request, request_size);
    // printf("%s: wrote request packet to Serial3\n", request_name);
    // receive response descriptor
    // printf("reading %u bytes from Serial3:\n", sizeof(response_descriptor));
    if (READ(&response_descriptor, sizeof(response_descriptor)) != eRpLidarOK)
        printf("ERROR: %s: cannot read response descriptor\n", request_name);
    // print_bytes(&response_descriptor, sizeof(response_descriptor));
    if (!(response_descriptor.start_flag1==START_FLAG1 && response_descriptor.start_flag2==START_FLAG2))
        printf("ERROR: %s: invalid format on RPLidar response descriptor\n", request_name);
    // print stuff to make sure we are parsing correctly
    // printf("%s: received response descriptor\n", request_name);
    printf("%s: response descriptor:\n", request_name);
    print_RpLidarResponseDescriptor(4, response_descriptor);
    if (response_descriptor.data_response_length != sizeof(RpLidarExpressScanDataResponseRaw))
        printf("ERROR: %s: response_descriptor.data_response_length != expected_data_response_length\n", request_name);
    // wait for motor speed to pick up
    // vTaskDelayUntil(&last_wake_time, 2000);
    // receive data response packet, repeat as many times as you want.
    const uint32_t express_scans_to_poll = 2;
    RpLidarExpressScanDataResponseRaw express_scan_data_response_raw;
    // response_descriptor.data_response_length = sizeof(express_scan_data_response_raw);
    for (uint32_t i = 0; i < express_scans_to_poll; ++i) {
        // printf("%s(%lu): reading %u bytes from Serial3:\n", request_name, i, response_descriptor.data_response_length);
        while (stream_read_exact(pHndl->rx_hndl, &express_scan_data_response_raw, response_descriptor.data_response_length, 100) != eRpLidarOK)
            printf("ERROR: %s(%lu): cannot read data response from Serial3\n", request_name, i);
        // print_bytes(&express_scan_data_response_raw, sizeof(express_scan_data_response_raw));
        // printf("%s(%lu): received data response\n", request_name, i);
        printf("%s(%lu): data response:\n", request_name, i);
        print_RpLidarExpressScanDataResponseRaw(4, express_scan_data_response_raw);
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



    for (;;) {
        printf("vRpLidar_tsk\n");
        vTaskDelayUntil(&last_wake_time, 1000);
    }
}
