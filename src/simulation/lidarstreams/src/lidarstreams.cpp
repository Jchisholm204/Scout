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

#include "lidarstreams/lidarstreams.hpp"

#include <cfloat>

LidarStreams::LidarStreams() : Node("lidarstreams") {
    this->declare_parameter("lidar_baud", 460800);
    this->declare_parameter("lidar_front_name", "lidar_front_frame");
    this->declare_parameter("lidar_vertical_name", "lidar_vertical_frame");
    this->declare_parameter("pub_rate", 500);
    this->declare_parameter("lidar_mode", 0);

    std::string lidar_front_name = this->get_parameter("lidar_front_name").as_string();
    std::string lidar_vertical_name =
        this->get_parameter("lidar_vertical_name").as_string();
    int64_t pub_rate = this->get_parameter("pub_rate").as_int();
    int64_t lidar_mode = this->get_parameter("lidar_mode").as_int();

    // Sanitize Parameters
    if (pub_rate < 0) {
        throw std::runtime_error("Publish Rate must be greater than 0");
    }
    if (lidar_mode < 0) {
        throw std::runtime_error("lidar_mode must be greater than 0");
    }

    // Initialize LiDAR

    _ls_front_pub =
        this->create_publisher<sensor_msgs::msg::LaserScan>(lidar_front_name, 10);
    _ls_vertical_pub =
        this->create_publisher<sensor_msgs::msg::LaserScan>(lidar_vertical_name, 10);

    _lidar_timer =
        this->create_wall_timer(std::chrono::milliseconds(pub_rate),
                                std::bind(&LidarStreams::lidar_callback, this));
};

LidarStreams::~LidarStreams() {
}

void LidarStreams::lidar_callback(void) {
    RCLCPP_INFO(this->get_logger(), "Lidar Callback!\n");
    sensor_msgs::msg::LaserScan msg;
    msg.angle_max = 0;
    msg.angle_min = FLT_MAX;
    msg.range_max = 0;
    msg.range_min = FLT_MAX;
    msg.header.stamp = this->now();

    msg.angle_min = 0;
    msg.angle_max = 2 * M_PI;
    msg.ranges.reserve(180);
    msg.intensities.reserve(180);

    for (size_t i = 0; i < 180; i++) {
        float distance = 1.1;
        msg.ranges.push_back(distance);
        msg.intensities.push_back(1);
    }

    msg.range_min = 0.1;
    msg.range_max = 6.0;
    msg.angle_increment = (msg.angle_max - msg.angle_min) / (msg.ranges.size() - 1);

    msg.header.frame_id = "lidar_front_frame";
    _ls_front_pub->publish(msg);
    msg.header.frame_id = "lidar_vertical_frame";
    _ls_vertical_pub->publish(msg);
}
