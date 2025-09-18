/**
 * @file rplidar_bridge.cpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief 
 * @version 0.1
 * @date Created: 2025-09-18
 * @modified Last Modified: 2025-09-18
 *
 * @copyright Copyright (c) 2025
 */

#include "rplidar_bridge/rplidar_bridge.hpp"

RPLIDARBridge::RPLIDARBridge() : Node("rplidar_bridge") {
    ///  Create a communication channel instance
    sl::IChannel* _channel;
    _channel = (*sl::createSerialPortChannel("/dev/ttyUSB0", 460800));
    ///  Create a LIDAR driver instance
    _lidar = *sl::createLidarDriver();
    auto res = (*_lidar).connect(_channel);
    if(SL_IS_OK(res)){
        sl_lidar_response_device_info_t deviceInfo;
        res = (*_lidar).getDeviceInfo(deviceInfo);
        if(SL_IS_OK(res)){
            printf("Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
            deviceInfo.model,
            deviceInfo.firmware_version >> 8, deviceInfo.firmware_version & 0xffu,
            deviceInfo.hardware_version);
        }else{
            fprintf(stderr, "Failed to get device information from LIDAR %08x\r\n", res);
        }
    }else{
        fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
    }
    _timer =
    this->create_wall_timer(std::chrono::milliseconds(50),
                            std::bind(&RPLIDARBridge::timer_callback, this));

    // Spin the lidar
    _lidar->setMotorSpeed();
    std::vector<sl::LidarScanMode> scan_modes;
    _lidar->getAllSupportedScanModes(scan_modes);
    _lidar->startScanExpress(false, scan_modes[0].id);
};

RPLIDARBridge::~RPLIDARBridge(){
    _lidar->stop();
    delete _lidar;
}

void RPLIDARBridge::timer_callback(void){
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t nodeCount = sizeof(nodes)/sizeof(sl_lidar_response_measurement_node_hq_t);
    auto res = _lidar->grabScanDataHq(nodes, nodeCount);
    if(SL_IS_OK(res)){
        for(size_t i = 0; i < nodeCount; i++){
            float angle = nodes[i].angle_z_q14 * 90.0f / (1 << 14);
            if(angle < 5){
                int distance = nodes[i].dist_mm_q2 * 100 / 1000 / (1<<2);
                std::cout << "angle:" << (int)(angle) << " distance: " << distance << " quality: " << (int)nodes[i].quality << std::endl;
            }
        }
    }

}
