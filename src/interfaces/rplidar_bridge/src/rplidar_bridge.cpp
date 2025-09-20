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

#include <cfloat>

RPLIDARBridge::RPLIDARBridge() : Node("rplidar_bridge") {
    this->declare_parameter("lidar_path", "/dev/ttyUSB0");
    this->declare_parameter("lidar_baud", 460800);
    this->declare_parameter("pub_prefix", "lidar");
    this->declare_parameter("frame_id", "map");
    this->declare_parameter("pub_rate", 50);
    this->declare_parameter("lidar_mode", 0);

    std::string lidar_path = this->get_parameter("lidar_path").as_string();
    int64_t lidar_baud = this->get_parameter("lidar_baud").as_int();
    std::string pub_prefix = this->get_parameter("pub_prefix").as_string();
    _frame_id = this->get_parameter("frame_id").as_string();
    int64_t pub_rate = this->get_parameter("pub_rate").as_int();
    int64_t lidar_mode = this->get_parameter("lidar_mode").as_int();

    // Sanitize Parameters
    if (access(lidar_path.c_str(), F_OK) == -1) {
        throw std::runtime_error("Lidar Path does not exist");
    }
    if (lidar_baud < 0) {
        throw std::runtime_error("Baud Must be greater than 0");
    }
    if (pub_rate < 0) {
        throw std::runtime_error("Publish Rate must be greater than 0");
    }
    if (lidar_mode < 0) {
        throw std::runtime_error("lidar_mode must be greater than 0");
    }

    // Initialize LiDAR

    ///  Create a communication channel instance
    sl::IChannel* _channel;
    _channel = (*sl::createSerialPortChannel(lidar_path, lidar_baud));
    ///  Create a LIDAR driver instance
    _lidar = *sl::createLidarDriver();
    auto res = (*_lidar).connect(_channel);
    if (SL_IS_OK(res)) {
        res = (*_lidar).getDeviceInfo(_lidar_info);
        if (SL_IS_OK(res)) {
            printf("Model: %d, Firmware Version: %d.%d, Hardware Version: %d\n",
                   _lidar_info.model, _lidar_info.firmware_version >> 8,
                   _lidar_info.firmware_version & 0xffu, _lidar_info.hardware_version);
        } else {
            fprintf(stderr, "Failed to get device information from LIDAR %08x\r\n", res);
            throw std::runtime_error("Lidar Connection Failure");
        }
    } else {
        fprintf(stderr, "Failed to connect to LIDAR %08x\r\n", res);
        throw std::runtime_error("Lidar Connection Failure");
    }

    _ls_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(pub_prefix, 10);

    _lidar_timer =
        this->create_wall_timer(std::chrono::milliseconds(pub_rate),
                                std::bind(&RPLIDARBridge::lidar_callback, this));

    // Start the lidar
    this->_start(true, lidar_mode);
};

RPLIDARBridge::~RPLIDARBridge() {
    _lidar->stop();
    delete _lidar;
}

void RPLIDARBridge::_start(bool on, int mode) {
    if (on) {
        // Spin the lidar
        _lidar->setMotorSpeed();
        std::vector<sl::LidarScanMode> scan_modes;
        _lidar->getAllSupportedScanModes(scan_modes);
        if ((size_t) mode >= scan_modes.size()) {
            throw std::invalid_argument("_start called with invalid mode");
        }
        _lidar->startScanExpress(false, scan_modes[mode].id);
    } else {
        _lidar->stop();
    }
}

void RPLIDARBridge::lidar_callback(void) {
    sl_lidar_response_measurement_node_hq_t nodes[8192];
    size_t nodeCount = sizeof(nodes) / sizeof(sl_lidar_response_measurement_node_hq_t);
    auto res = _lidar->grabScanDataHq(nodes, nodeCount);
    sensor_msgs::msg::LaserScan msg;
    msg.angle_max = 0;
    msg.angle_min = FLT_MAX;
    msg.range_max = 0;
    msg.range_min = FLT_MAX;
    msg.header.stamp = this->now();
    msg.header.frame_id = _frame_id;
    if (SL_IS_OK(res)) {
        // Sort nodes by angle (ascending)
        std::sort(nodes, nodes + nodeCount, [](const auto& a, const auto& b) {
            float angleA = a.angle_z_q14 * 90.0f / (1 << 14);
            float angleB = b.angle_z_q14 * 90.0f / (1 << 14);
            return angleA > angleB;
        });

        msg.angle_min = FLT_MAX;
        msg.angle_max = 0;
        msg.ranges.reserve(nodeCount);
        msg.intensities.reserve(nodeCount);

        for (size_t i = 0; i < nodeCount; i++) {
            float angle = nodes[i].angle_z_q14 * 90.0f / (1 << 14);
            angle = angle * M_PI / 180;
            if (angle < msg.angle_min)
                msg.angle_min = angle;
            if (angle > msg.angle_max)
                msg.angle_max = angle;
            float distance = ((float) nodes[i].dist_mm_q2) / 1000.0f / (float) (1 << 2);
            msg.ranges.push_back(distance);
            msg.intensities.push_back(nodes[i].quality);
        }

        msg.range_min = 0.1;
        msg.range_max = 6.0;
        msg.angle_increment = (msg.angle_max - msg.angle_min) / (msg.ranges.size() - 1);

        _ls_pub->publish(msg);
    }

    // if (SL_IS_OK(res)) {
    //     for (size_t i = 0; i < nodeCount; i++) {
    //         float angle = nodes[i].angle_z_q14 * 90.0f / (1 << 14);
    //         angle = angle * M_PI / 180;
    //         if (angle < msg.angle_min)
    //             msg.angle_min = angle;
    //         if (angle > msg.angle_max)
    //             msg.angle_max = angle;
    //         float distance =
    //             ((float) nodes[i].dist_mm_q2) / 1000.0f / (float) (1 << 2);
    //         msg.ranges.push_back(distance);
    //         msg.intensities.push_back(nodes[i].quality);
    //     }
    //     msg.range_min = 0.1;
    //     msg.range_max = 6.0;
    //     msg.angle_increment = (msg.angle_max - msg.angle_min) / msg.ranges.size();
    //     _ls_pub->publish(msg);
    // }
}
