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

static void print_RpLidarRequestWithPayload(uint16_t indent, RpLidarRequestWithPayload rwp) {
    printf("%*sstart_flag: 0x%02X\n", indent, "", rwp.start_flag);
    printf("%*scommand: 0x%02X\n", indent, "", rwp.command);
    printf("%*spayload_size: 0x%02X\n", indent, "", rwp.payload_size);
    printf("%*spayload:", indent, "");
    uint8_t i = 0;
    do {
        printf(" %02X", rwp.payload[i]);
    } while (i < rwp.payload_size);
    printf("\n");
    printf("%*spayload_size: 0x%02X\n", indent, "", rwp.checksum);
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
    printf("%*sstatus: %X\n", indent, "", device_health.status);
    printf("%*serror_code: %X\n", indent, "", device_health.error_code);
}

static void print_RpLidarSampleRate(uint16_t indent, RpLidarSampleRate sample_rate) {
    printf("%*sTstandard: %u\n", indent, "", sample_rate.Tstandard);
    printf("%*sTexpress: %u\n", indent, "", sample_rate.Texpress);
}

static void print_RpLidarConfResponseDataWithU8Payload(const uint16_t indent, const RpLidarConfResponseDataWithU8Payload data, const char *const payload_name) {
    printf("%*stype: %lu\n", indent, "", data.type);
    printf("%*s%s: %u\n", indent, "", payload_name, data.payload);
}

static void print_RpLidarConfResponseDataWithU16Payload(const uint16_t indent, const RpLidarConfResponseDataWithU16Payload data, const char *const payload_name) {
    printf("%*stype: %lu\n", indent, "", data.type);
    printf("%*s%s: %u\n", indent, "", payload_name, data.payload);
}

static void print_RpLidarConfResponseDataWithU32Payload(const uint16_t indent, const RpLidarConfResponseDataWithU32Payload data, const char *const payload_name) {
    printf("%*stype: %lu\n", indent, "", data.type);
    printf("%*s%s: %lu\n", indent, "", payload_name, data.payload);
}

static void print_RpLidarConfResponseDataWithStringPayload(const uint16_t indent, const RpLidarConfResponseDataWithStringPayload data, const char *const payload_name) {
    printf("%*stype: %lu\n", indent, "", data.type);
    printf("%*s%s: %s\n", indent, "", payload_name, data.payload);
}

static uint8_t checksum_RpLidarRequestPacketWithPayload(RpLidarRequestWithPayload *request_packet) {
    uint8_t checksum = request_packet->start_flag ^ request_packet->command ^ request_packet->payload_size;
    uint8_t *payload = request_packet->payload;
    for (uint16_t i = 0; i < request_packet->payload_size; ++i)
        checksum ^= payload[i];
    return checksum;
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
    const char *request_name;
    RpLidarRequestNoPayload request_packet_no_payload = {.start_flag=START_FLAG};
    RpLidarRequestWithPayload request_packet_with_payload;
    RpLidarResponseDescriptor response_descriptor;
    char request[32];
    TickType_t last_wake_time;

    #define READ(var) stream_read_exact(pHndl->rx_hndl, (void *)&var, sizeof(var), 10)
    #define WRITE(var) serial_write(pHndl->pSerial, (char *)&var, sizeof(var), 10) 



    // GET_INFO
    request_name = "GET_INFO";
    // prepare GET_INFO request
    // send GET_INFO request
    // receive GET_INFO response descriptor
    // receive GET_INFO data response
    
    // prepare GET_INFO request
    request_packet_no_payload.command=COMMAND_GET_INFO;
    // print to make sure it's prepared correctly
    printf("%s: request packet: \n", request_name);
    print_RpLidarRequestNoPayload(4, request_packet_no_payload);
    // send GET_INFO request
    printf("writing %u bytes to Serial3:\n", sizeof(request_packet_no_payload));
    if (WRITE(request_packet_no_payload) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // print stuff so we know what was sent
    print_bytes(&request_packet_no_payload, sizeof(request_packet_no_payload));
    printf("%s: wrote request packet to Serial3\n", request_name);
    // receive GET_INFO response descriptor
    printf("reading %u bytes from Serial3:\n", sizeof(response_descriptor));
    if (READ(response_descriptor) != eRpLidarOK)
        printf("ERROR: cannot read GET_INFO response descriptor\n");
    if (!(response_descriptor.start_flag1==START_FLAG1 && response_descriptor.start_flag2==START_FLAG2))
        printf("ERROR: invalid format on RPLidar response\n");
    // print stuff to make sure we are parsing correctly
    print_bytes(&response_descriptor, sizeof(response_descriptor));
    printf("%s: received response descriptor\n", request_name);
    printf("%s: response descriptor:\n", request_name);
    print_RpLidarResponseDescriptor(4, response_descriptor);
    if (response_descriptor.data_response_length != sizeof(RpLidarDeviceInfo))
        printf("ERROR: %s: response_descriptor.data_response_length != sizeof(RpLidarDeviceInfo)\n", request_name);
    // receive GET_INFO data response
    RpLidarDeviceInfo device_info;
    printf("%s: reading %u bytes from Serial3:\n", request_name, sizeof(device_info));
    if (READ(device_info) != eRpLidarOK)
        printf("ERROR: %s: cannot read data response\n", request_name);
    // print stuff to make sure we are parsing correctly
    print_bytes(&device_info, sizeof(device_info));
    printf("%s: received data response\n", request_name);
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
    if (WRITE(request_packet_no_payload) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    // receive GET_HEALTH response descriptor
    if (READ(response_descriptor) != eRpLidarOK)
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
    if (READ(device_health) != eRpLidarOK)
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
    // send GET_SAMPLERATE request
    if (WRITE(request_packet_no_payload) != eSerialOK)
        printf("ERROR: %s: cannot write request to Serial3\n", request_name);
    // receive GET_SAMPLERATE response descriptor
    if (READ(response_descriptor) != eRpLidarOK)
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
    READ(sample_rate);
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
    RpLidarConfRequestDataNoPayload request_data = {.type=RPLIDAR_CONF_SCAN_MODE_COUNT};
    memcpy(request_packet_with_payload.payload, &request_data, request_packet_with_payload.payload_size);
    request_packet_with_payload.checksum = checksum_RpLidarRequestPacketWithPayload(&request_packet_with_payload);
    memcpy(request, &request_packet_with_payload, offsetof(typeof(request_packet_with_payload), payload));
    memcpy(request+offsetof(typeof(request_packet_with_payload), payload), request_packet_with_payload.payload, request_packet_with_payload.payload_size);
    memcpy(request+offsetof(typeof(request_packet_with_payload), payload) + request_packet_with_payload.payload_size, &request_packet_with_payload.checksum, sizeof(request_packet_with_payload.checksum));
    uint16_t request_size = sizeof(RpLidarRequestWithPayload) - 255U + request_packet_with_payload.payload_size;
    printf("%s: request packet:\n", request_name);
    print_RpLidarRequestWithPayload(4, request_packet_with_payload);
    // send request packet
    printf("writing %u bytes to Serial3:\n", request_size);
    if (serial_write(pHndl->pSerial, request, request_size, 10) != eSerialOK)
        printf("ERROR: %s: cannot write request packet to Serial3\n", request_name);
    print_bytes(request, request_size);
    printf("%s: wrote request packet to Serial3\n", request_name);
    // receive response descriptor
    // print stuff to make sure we are parsing correctly
    printf("reading %u bytes from Serial3:\n", sizeof(RpLidarResponseDescriptor));
    print_bytes(&response_descriptor, sizeof(response_descriptor));
    printf("%s: response descriptor:\n", request_name);
    print_RpLidarResponseDescriptor(4, response_descriptor);
    if (READ(response_descriptor) != eRpLidarOK)
        printf("ERROR: %s: cannot read response descriptor from Serial3\n", request_name);
    if (response_descriptor.data_response_length != sizeof(RpLidarConfResponseDataWithU16Payload))
        printf("ERROR: %s: response_descriptor.data_response_length != expected_data_response_length\n", request_name);
    // receive data response
    RpLidarConfResponseDataWithU16Payload scan_mode_count_data_response;
    if (READ(scan_mode_count_data_response) != eRpLidarOK)
        printf("ERROR: %s: cannot read data response from Serial3\n", request_name);
    // print stuff to make sure we are parsing correctly
    printf("%s: data response:\n", request_name);

    
    #if 0
    // RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE (0x71)
    // Get microsecond cost per measurement sample for specific scan mode (in Q8 fixed point format). Actual value = laser_range_time_q8/256.0 (microseconds).
    // prepare RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE request
    // send RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE request
    // receive RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE response descriptor
    // receive RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE data response

    if(1) {
    for (uint16_t mode = 0; mode < scan_mode_count; ++mode) {
        printf("RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE request: mode: %u\n", mode);
        // prepare RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE request
        uint8_t command = 0x84;
        uint8_t request_payload_size;
        struct {
            uint32_t type;
            uint16_t payload;
        } __attribute__((packed)) request_payload_data = {.type=0x71, .payload=mode};
        request_payload_size = sizeof(request_payload_data);
        uint8_t checksum = 0;
        uint16_t request_size = 0;
        memcpy(request+request_size, &start_flag, sizeof(start_flag)), request_size+=sizeof(start_flag);
        memcpy(request+request_size, &command, sizeof(command)), request_size+=sizeof(command);
        memcpy(request+request_size, &request_payload_size, sizeof(request_payload_size)), request_size+=sizeof(request_payload_size);
        memcpy(request+request_size, &request_payload_data, sizeof(request_payload_data)), request_size+=sizeof(request_payload_data);
        for (uint16_t i = 0; i < request_size; ++i)
            checksum ^= request[i];
        memcpy(request+request_size, &checksum, sizeof(checksum)), request_size+=sizeof(checksum);
        // send RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE request
        printf("writing %d bytes to Serial3:\n", request_size);
        if (serial_write(pHndl->pSerial, request, request_size, 10) != eSerialOK){
            printf("cannot write RPLIDAR_CONF_SCAN_MODE_COUNT request to Serial3\n");
        }
        for (u_int i = 0; i < request_size; ++i) {
            printf("byte %u: %X\n", i, request[i]);
        }
        printf("wrote RPLIDAR_CONF_SCAN_MODE_COUNT request to Serial3\n");
        // receive RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE response descriptor
        RpLidarResponseDescriptor rplidar_conf_scan_mode_us_per_sample_response_descriptor;
        printf("reading %u bytes from Serial3:\n", sizeof(RpLidarResponseDescriptor));
        for (u_int i = 0; i < sizeof(RpLidarResponseDescriptor);) {
            if (xStreamBufferReceive(pHndl->rx_hndl, response+i, 1, 10)) {
                printf("byte %d: %X\n", i, response[i]);
                i++;
            }
        }
        if (!(response[0]==RPLIDAR_RESPONSE_DESCRIPTOR.start_flag1 && response[1]==RPLIDAR_RESPONSE_DESCRIPTOR.start_flag2)) {
            printf("error: invalid format on RPLidar response\n");
        }
        printf("RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE response descriptor: %X %X %X %X %X %X %X\n", response[0], response[1], response[2], response[3], response[4], response[5], response[6]);
        memcpy(&rplidar_conf_scan_mode_us_per_sample_response_descriptor, response, sizeof(rplidar_conf_scan_mode_us_per_sample_response_descriptor));
        // print stuff to make sure we are parsing correctly
        printf("rplidar_conf_scan_mode_us_per_sample_response_descriptor:\n");
        printf("    start_flag1: %X\n", rplidar_conf_scan_mode_us_per_sample_response_descriptor.start_flag1);
        printf("    start_flag2: %X\n", rplidar_conf_scan_mode_us_per_sample_response_descriptor.start_flag2);
        printf("    data_response_length: %u\n", rplidar_conf_scan_mode_us_per_sample_response_descriptor.data_response_length);
        printf("    send_mode: %X\n", rplidar_conf_scan_mode_us_per_sample_response_descriptor.send_mode);
        printf("    data_type: %X\n", rplidar_conf_scan_mode_us_per_sample_response_descriptor.data_type);
        if (rplidar_conf_scan_mode_us_per_sample_response_descriptor.data_response_length != sizeof(uint32_t) + sizeof(uint32_t)) {
            printf("ERROR: rplidar_conf_scan_mode_us_per_sample_response_descriptor.data_response_length != sizeof(uint32_t) + sizeof(uint32_t)\n");
        }
        // receive RPLIDAR_CONF_SCAN_MODE_US_PER_SAMPLE data response
        uint32_t rplidar_conf_scan_mode_us_per_sample_response_descriptor_data_response_type;
        uint32_t laser_range_time_q8;
        data_response_length = rplidar_conf_scan_mode_us_per_sample_response_descriptor.data_response_length;
        printf("reading %lu bytes from Serial3:\n", data_response_length);
        for (uint32_t i = 0; i < data_response_length;) {
            if (xStreamBufferReceive(pHndl->rx_hndl, response+i, 1, 10)) {
                printf("byte %lu: %X\n", i, response[i]);
                i++;
            }
        }
        memcpy(&rplidar_conf_scan_mode_us_per_sample_response_descriptor_data_response_type, response, sizeof(uint32_t));
        memcpy(&laser_range_time_q8, response+sizeof(uint32_t), sizeof(uint32_t));
        // print stuff to make sure we are parsing correctly
        printf("rplidar_conf_scan_mode_us_per_sample_response_descriptor_data_response_type: 0x%lX\n", rplidar_conf_scan_mode_us_per_sample_response_descriptor_data_response_type);
        printf("laser_range_time_q8: %lu\n", laser_range_time_q8);
    }
    }

    // RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE (0x74)
    // Get max measurement distance for specific scan mode (in m, Q8 fixed point format). Actual Value = max_distance_q8/256.0 (meters).
    // prepare RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE request
    // send RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE request
    // receive RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE response descriptor
    // receive RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE data response

    if(0) {
    for (uint16_t mode = 0; mode < scan_mode_count; ++mode) {
        printf("RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE request: mode: %u\n", mode);
        // prepare RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE request
        uint8_t command = 0x84;
        uint8_t request_payload_size;
        struct {
            uint32_t type;
            uint16_t payload;
        } __attribute__((packed)) request_payload_data = {.type=0x74, .payload=mode};
        request_payload_size = sizeof(request_payload_data);
        uint8_t checksum = 0;
        uint16_t request_size = 0;
        memcpy(request+request_size, &start_flag, sizeof(start_flag)), request_size+=sizeof(start_flag);
        memcpy(request+request_size, &command, sizeof(command)), request_size+=sizeof(command);
        memcpy(request+request_size, &request_payload_size, sizeof(request_payload_size)), request_size+=sizeof(request_payload_size);
        memcpy(request+request_size, &request_payload_data, sizeof(request_payload_data)), request_size+=sizeof(request_payload_data);
        for (uint16_t i = 0; i < request_size; ++i)
            checksum ^= request[i];
        memcpy(request+request_size, &checksum, sizeof(checksum)), request_size+=sizeof(checksum);
        // send RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE request
        printf("writing %d bytes to Serial3:\n", request_size);
        if (serial_write(pHndl->pSerial, request, request_size, 10) != eSerialOK){
            printf("cannot write RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE request to Serial3\n");
        }
        for (u_int i = 0; i < request_size; ++i) {
            printf("byte %u: %X\n", i, request[i]);
        }
        printf("wrote RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE request to Serial3\n");
        // receive RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE response descriptor
        RpLidarResponseDescriptor rplidar_conf_scan_mode_max_distance_response_descriptor;
        printf("reading %u bytes from Serial3:\n", sizeof(RpLidarResponseDescriptor));
        for (u_int i = 0; i < sizeof(RpLidarResponseDescriptor);) {
            if (xStreamBufferReceive(pHndl->rx_hndl, response+i, 1, 10)) {
                printf("byte %d: %X\n", i, response[i]);
                i++;
            }
        }
        if (!(response[0]==RPLIDAR_RESPONSE_DESCRIPTOR.start_flag1 && response[1]==RPLIDAR_RESPONSE_DESCRIPTOR.start_flag2)) {
            printf("error: invalid format on RPLidar response\n");
        }
        printf("RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE response descriptor: %X %X %X %X %X %X %X\n", response[0], response[1], response[2], response[3], response[4], response[5], response[6]);
        memcpy(&rplidar_conf_scan_mode_max_distance_response_descriptor, response, sizeof(rplidar_conf_scan_mode_max_distance_response_descriptor));
        // print stuff to make sure we are parsing correctly
        printf("rplidar_conf_scan_mode_max_distance_response_descriptor:\n");
        printf("    start_flag1: %X\n", rplidar_conf_scan_mode_max_distance_response_descriptor.start_flag1);
        printf("    start_flag2: %X\n", rplidar_conf_scan_mode_max_distance_response_descriptor.start_flag2);
        printf("    data_response_length: %u\n", rplidar_conf_scan_mode_max_distance_response_descriptor.data_response_length);
        printf("    send_mode: %X\n", rplidar_conf_scan_mode_max_distance_response_descriptor.send_mode);
        printf("    data_type: %X\n", rplidar_conf_scan_mode_max_distance_response_descriptor.data_type);
        if (rplidar_conf_scan_mode_max_distance_response_descriptor.data_response_length != sizeof(uint32_t) + sizeof(uint32_t)) {
            printf("ERROR: rplidar_conf_scan_mode_max_distance_response_descriptor.data_response_length != sizeof(uint32_t) + sizeof(uint32_t)\n");
        }
        // receive RPLIDAR_CONF_SCAN_MODE_MAX_DISTANCE data response
        uint32_t rplidar_conf_scan_mode_max_distance_response_descriptor_data_response_type;
        uint32_t max_distance_q8;
        data_response_length = rplidar_conf_scan_mode_max_distance_response_descriptor.data_response_length;
        printf("reading %lu bytes from Serial3:\n", data_response_length);
        for (uint32_t i = 0; i < data_response_length;) {
            if (xStreamBufferReceive(pHndl->rx_hndl, response+i, 1, 10)) {
                printf("byte %lu: %X\n", i, response[i]);
                i++;
            }
        }
        memcpy(&rplidar_conf_scan_mode_max_distance_response_descriptor_data_response_type, response, sizeof(uint32_t));
        memcpy(&max_distance_q8, response+sizeof(uint32_t), sizeof(uint32_t));
        // print stuff to make sure we are parsing correctly
        printf("rplidar_conf_scan_mode_max_distance_response_descriptor_data_response_type: 0x%lX\n", rplidar_conf_scan_mode_max_distance_response_descriptor_data_response_type);
        printf("max_distance_q8: %lu\n", max_distance_q8);
    }
    }

    // RPLIDAR_CONF_SCAN_MODE_ANS_TYPE (0x75)
    // prepare RPLIDAR_CONF_SCAN_MODE_ANS_TYPE request
    // send RPLIDAR_CONF_SCAN_MODE_ANS_TYPE request
    // receive RPLIDAR_CONF_SCAN_MODE_ANS_TYPE response descriptor
    // receive RPLIDAR_CONF_SCAN_MODE_ANS_TYPE data response
    if (0) {
    for (uint16_t mode = 0; mode < scan_mode_count; ++mode) {
        printf("RPLIDAR_CONF_SCAN_MODE_ANS_TYPE request: mode: %u\n", mode);
        // prepare RPLIDAR_CONF_SCAN_MODE_ANS_TYPE request
        uint8_t command = 0x84;
        uint8_t request_payload_size;
        struct {
            uint32_t type;
            uint16_t payload;
        } __attribute__((packed)) request_payload_data = {.type=0x75, .payload=mode};
        request_payload_size = sizeof(request_payload_data);
        uint8_t checksum = 0;
        uint16_t request_size = 0;
        memcpy(request+request_size, &start_flag, sizeof(start_flag)), request_size+=sizeof(start_flag);
        memcpy(request+request_size, &command, sizeof(command)), request_size+=sizeof(command);
        memcpy(request+request_size, &request_payload_size, sizeof(request_payload_size)), request_size+=sizeof(request_payload_size);
        memcpy(request+request_size, &request_payload_data, sizeof(request_payload_data)), request_size+=sizeof(request_payload_data);
        for (uint16_t i = 0; i < request_size; ++i)
            checksum ^= request[i];
        memcpy(request+request_size, &checksum, sizeof(checksum)), request_size+=sizeof(checksum);
        // send RPLIDAR_CONF_SCAN_MODE_ANS_TYPE request
        printf("writing %d bytes to Serial3:\n", request_size);
        if (serial_write(pHndl->pSerial, request, request_size, 10) != eSerialOK){
            printf("cannot write RPLIDAR_CONF_SCAN_MODE_ANS_TYPE request to Serial3\n");
        }
        for (u_int i = 0; i < request_size; ++i) {
            printf("byte %u: %X\n", i, request[i]);
        }
        printf("wrote RPLIDAR_CONF_SCAN_MODE_ANS_TYPE request to Serial3\n");
        // receive RPLIDAR_CONF_SCAN_MODE_ANS_TYPE response descriptor
        RpLidarResponseDescriptor rplidar_conf_scan_mode_ans_type_response_descriptor;
        printf("reading %u bytes from Serial3:\n", sizeof(RpLidarResponseDescriptor));
        for (u_int i = 0; i < sizeof(RpLidarResponseDescriptor);) {
            if (xStreamBufferReceive(pHndl->rx_hndl, response+i, 1, 10)) {
                printf("byte %d: %X\n", i, response[i]);
                i++;
            }
        }
        if (!(response[0]==RPLIDAR_RESPONSE_DESCRIPTOR.start_flag1 && response[1]==RPLIDAR_RESPONSE_DESCRIPTOR.start_flag2)) {
            printf("error: invalid format on RPLidar response\n");
        }
        printf("RPLIDAR_CONF_SCAN_MODE_ANS_TYPE response descriptor: %X %X %X %X %X %X %X\n", response[0], response[1], response[2], response[3], response[4], response[5], response[6]);
        memcpy(&rplidar_conf_scan_mode_ans_type_response_descriptor, response, sizeof(rplidar_conf_scan_mode_ans_type_response_descriptor));
        // print stuff to make sure we are parsing correctly
        printf("rplidar_conf_scan_mode_ans_type_response_descriptor:\n");
        printf("    start_flag1: %X\n", rplidar_conf_scan_mode_ans_type_response_descriptor.start_flag1);
        printf("    start_flag2: %X\n", rplidar_conf_scan_mode_ans_type_response_descriptor.start_flag2);
        printf("    data_response_length: %u\n", rplidar_conf_scan_mode_ans_type_response_descriptor.data_response_length);
        printf("    send_mode: %X\n", rplidar_conf_scan_mode_ans_type_response_descriptor.send_mode);
        printf("    data_type: %X\n", rplidar_conf_scan_mode_ans_type_response_descriptor.data_type);
        if (rplidar_conf_scan_mode_ans_type_response_descriptor.data_response_length != sizeof(uint32_t) + sizeof(uint32_t)) {
            printf("ERROR: rplidar_conf_scan_mode_ans_type_response_descriptor.data_response_length != sizeof(uint32_t) + sizeof(uint32_t)\n");
        }
        // receive RPLIDAR_CONF_SCAN_MODE_ANS_TYPE data response
        uint32_t rplidar_conf_scan_mode_ans_type_response_descriptor_data_response_type;
        uint8_t ans_type; // Typical return answer types: 
        // 0x81 –In this mode, the scan range data is returned using SCAN command corresponding to the response data format 
        // 0x82 – In this mode, the scan range data is returned using EXPRESS_SCAN command corresponding to the traditional version format 
        // 0x83 – In this mode, the scan range data is returned using EXPRESS_SCAN command corresponding to the ultra capsuled format
        data_response_length = rplidar_conf_scan_mode_ans_type_response_descriptor.data_response_length;
        printf("reading %lu bytes from Serial3:\n", data_response_length);
        for (uint32_t i = 0; i < data_response_length;) {
            if (xStreamBufferReceive(pHndl->rx_hndl, response+i, 1, 10)) {
                printf("byte %lu: %X\n", i, response[i]);
                i++;
            }
        }
        memcpy(&rplidar_conf_scan_mode_ans_type_response_descriptor_data_response_type, response, sizeof(uint32_t));
        memcpy(&ans_type, response+sizeof(uint32_t), sizeof(uint8_t));
        // print stuff to make sure we are parsing correctly
        printf("rplidar_conf_scan_mode_ans_type_response_descriptor_data_response_type: 0x%lX\n", rplidar_conf_scan_mode_ans_type_response_descriptor_data_response_type);
        printf("ans_type: %X\n", ans_type);
    }
    }

    // RPLIDAR_CONF_SCAN_MODE_TYPICAL (0x7C)
    // prepare RPLIDAR_CONF_SCAN_MODE_TYPICAL request
    // send RPLIDAR_CONF_SCAN_MODE_TYPICAL request
    // receive RPLIDAR_CONF_SCAN_MODE_TYPICAL response descriptor
    // receive RPLIDAR_CONF_SCAN_MODE_TYPICAL data response 

    // if (1){
    // prepare RPLIDAR_CONF_SCAN_MODE_TYPICAL request
    request[0] = 0xA5;
    request[1] = 0x84;
    request[2] = 0x4;
    request[3] = 0x7C;
    request[4] = 0x0;
    request[5] = 0x0;
    request[6] = 0x0;
    request[7] = request[0] ^ request[1] ^ request[2] ^ request[3] ^ request[4] ^ request[5] ^ request[6];
    // send RPLIDAR_CONF_SCAN_MODE_TYPICAL request
    printf("writing 8 bytes to Serial3:\n");
    for (u_int i = 0; i < 8; ++i) {
        printf("byte %u: %X\n", i, request[i]);
    }
    if (serial_write(pHndl->pSerial, request, 8, 10) != eSerialOK){
        printf("cannot write RPLIDAR_CONF_SCAN_MODE_TYPICAL request to Serial3\n");
    }
    printf("wrote RPLIDAR_CONF_SCAN_MODE_TYPICAL request to Serial3\n");
    // receive RPLIDAR_CONF_SCAN_MODE_TYPICAL response descriptor
    RpLidarResponseDescriptor rplidar_conf_scan_mode_typical_response_descriptor;
    printf("reading %u bytes from Serial3:\n", sizeof(RpLidarResponseDescriptor));
    for (u_int i = 0; i < sizeof(RpLidarResponseDescriptor);) {
        if (xStreamBufferReceive(pHndl->rx_hndl, response+i, 1, 10)) {
            printf("byte %d: %X\n", i, response[i]);
            i++;
        }
    } 
    if (!(response[0]==RPLIDAR_RESPONSE_DESCRIPTOR.start_flag1 && response[1]==RPLIDAR_RESPONSE_DESCRIPTOR.start_flag2)) {
        printf("error: invalid format on RPLidar response\n");
    }
    printf("SCAN response descriptor: %X %X %X %X %X %X %X\n", response[0], response[1], response[2], response[3], response[4], response[5], response[6]);
    memcpy(&rplidar_conf_scan_mode_typical_response_descriptor, response, sizeof(rplidar_conf_scan_mode_typical_response_descriptor));
    // print stuff to make sure we are parsing correctly
    printf("scan_response_descriptor:\n");
    printf("    start_flag1: %X\n", rplidar_conf_scan_mode_typical_response_descriptor.start_flag1);
    printf("    start_flag2: %X\n", rplidar_conf_scan_mode_typical_response_descriptor.start_flag2);
    printf("    data_response_length: %u\n", rplidar_conf_scan_mode_typical_response_descriptor.data_response_length);
    printf("    send_mode: %X\n", rplidar_conf_scan_mode_typical_response_descriptor.send_mode);
    printf("    data_type: %X\n", rplidar_conf_scan_mode_typical_response_descriptor.data_type);
    if (rplidar_conf_scan_mode_typical_response_descriptor.data_response_length != sizeof(uint32_t) + sizeof(uint16_t)) {
        printf("ERROR: rplidar_conf_scan_mode_typical_response_descriptor.data_response_length != sizeof(uint32_t) + sizeof(uint16_t)\n");
    }
    // receive RPLIDAR_CONF_SCAN_MODE_TYPICAL data response 
    uint32_t rplidar_conf_scan_mode_typical_response_type;
    uint16_t scan_mode;
    data_response_length = rplidar_conf_scan_mode_typical_response_descriptor.data_response_length;
    printf("reading %lu bytes from Serial3:\n", data_response_length);
    for (uint32_t i = 0; i < data_response_length;) {
        if (xStreamBufferReceive(pHndl->rx_hndl, response+i, 1, 10)) {
            printf("byte %lu: %X\n", i, response[i]);
            i++;
        }
    }
    memcpy(&rplidar_conf_scan_mode_typical_response_type, response, sizeof(rplidar_conf_scan_mode_typical_response_type));
    memcpy(&scan_mode, response+4, sizeof(scan_mode));
    // print stuff to make sure we are parsing correctly
    printf("rplidar_conf_scan_mode_typical_response_type: 0x%lX\n", rplidar_conf_scan_mode_typical_response_type);
    printf("scan_mode: %u\n", scan_mode);
    // }

    // SCAN
    // prepare SCAN request
    // send SCAN request
    // receive SCAN response descriptor
    // The data response packets related to every measurement sample results will be sent out continuously only after the motor rotation becomes stable.
    // receive SCAN data response packet, repeat as many times as you want.

    // prepare SCAN request
    request_packet_no_payload.command = COMMAND_SCAN;
    request_size = sizeof(RpLidarRequestNoPayload);
    memcpy(request, &request_packet_no_payload, request_size);
    // send SCAN request
    printf("writing %u bytes to Serial3\n", request_size);
    for (uint16_t i = 0; i < request_size; ++i)
        printf("byte %u: %2X\n", i, request[i]);
    if (serial_write(pHndl->pSerial, request, request_size, 10) != eSerialOK){
        printf("cannot write SCAN request to Serial3\n");
    }
    printf("wrote SCAN request to Serial3\n");
    // receive SCAN response descriptor
    RpLidarResponseDescriptor scan_response_descriptor;
    printf("reading %u bytes from Serial3:\n", sizeof(RpLidarResponseDescriptor));
    for (u_int i = 0; i < sizeof(RpLidarResponseDescriptor);) {
        if (xStreamBufferReceive(pHndl->rx_hndl, response+i, 1, 10)) {
            printf("byte %d: %X\n", i, response[i]);
            i++;
        }
    } 
    if (!(response[0]==RPLIDAR_RESPONSE_DESCRIPTOR.start_flag1 && response[1]==RPLIDAR_RESPONSE_DESCRIPTOR.start_flag2)) {
        printf("error: invalid format on RPLidar response\n");
    }
    printf("SCAN response descriptor: %X %X %X %X %X %X %X\n", response[0], response[1], response[2], response[3], response[4], response[5], response[6]);
    memcpy(&scan_response_descriptor, response, sizeof(scan_response_descriptor));
    // print stuff to make sure we are parsing correctly
    printf("scan_response_descriptor:\n");
    printf("    start_flag1: %X\n", scan_response_descriptor.start_flag1);
    printf("    start_flag2: %X\n", scan_response_descriptor.start_flag2);
    printf("    data_response_length: %u\n", scan_response_descriptor.data_response_length);
    printf("    send_mode: %X\n", scan_response_descriptor.send_mode);
    printf("    data_type: %X\n", scan_response_descriptor.data_type);
    if (scan_response_descriptor.data_response_length != sizeof(RpLidarScan)) {
        printf("ERROR: scan_response_descriptor.data_response_length != sizeof(RpLidarScan)\n");
    }
    // receive SCAN data response packet, repeat as many times as you want.
    data_response_length = scan_response_descriptor.data_response_length;
    const uint32_t scans_to_poll = 10;
    for (uint32_t c = 0; c < scans_to_poll; c++) {
        printf("SCAN %lu:\n", c);
        RpLidarScan scan;
        printf("reading %lu bytes from Serial3:\n",  data_response_length);
        for (uint32_t i = 0; i < data_response_length;) {
            if (xStreamBufferReceive(pHndl->rx_hndl, response+i, 1, 10)) {
                printf("byte %lu: %X\n", i, response[i]);
                i++;
            }
        }
        memcpy(&scan, response, scan_response_descriptor.data_response_length);

        printf("scan%lu:\n", c);
        printf("    start: %u\n", scan.start);
        printf("    n_start: %u\n", scan.n_start);
        printf("    quality: %u\n", scan.quality);
        printf("    check: %u\n", scan.check);
        printf("    angle_q6: %d\n", scan.angle_q6);
        printf("    distance_q2: %d\n", scan.distance_q2);
    }


    // send a STOP request to stop the scanning
    // send a STOP request
    // wait at least 10ms
    request_packet_no_payload.command = COMMAND_STOP;
    request_size = sizeof(request_packet_no_payload);
    memcpy(request, &request_packet_no_payload, request_size);
    if (serial_write(pHndl->pSerial, request, request_size, 10) != eSerialOK){
        printf("cannot write STOP request to Serial3\n");
    }
    last_wake_time = xTaskGetTickCount();
    printf("wrote STOP request to Serial3\n");
    vTaskDelayUntil(&last_wake_time, 10);
    printf("at least 10 milliseconds have past since sending STOP request to Serial3\n");

    #endif



    last_wake_time = xTaskGetTickCount();
    for (;;) {
        printf("vRpLidar_tsk\n");
        vTaskDelayUntil(&last_wake_time, 1000);
    }
}
