/**
 * @file inav_bridge.cpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2025-09-17
 * @modified Last Modified: 2025-09-17
 *
 * @copyright Copyright (c) 2025
 */

#include "gravitational/gravitational.hpp"

#include <tf2/LinearMath/Quaternion.h>

#define ZERO_VEC(vec)                                                                    \
    (vec).x = 0;                                                                         \
    (vec).y = 0;                                                                         \
    (vec).z = 0;

#include <unistd.h>

Gravitational::Gravitational() : Node("gravity_ctrl") {
    this->declare_parameter("imu_topic", "/inav_bridge/imu");
    this->declare_parameter("gyro_topic", "/inav_bridge/gyro");
    this->declare_parameter("battery_topic", "/inav_bridge/batt");
    this->declare_parameter("motor_topic", "/inav_bridge/mtr");
    this->declare_parameter("altitude_topic", "/inav_bridge/altitude");
    this->declare_parameter("hor_lidar_topic", "/lidar");
    this->declare_parameter("vrt_lidar_topic", "/veridar");
    this->declare_parameter("target_topic", "/gtarget");
    this->declare_parameter("movement_topic", "/inav_bridge/movement");
    this->declare_parameter("arming_topic", "/inav_bridge/armed");

    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    std::string gyro_topic = this->get_parameter("gyro_topic").as_string();
    std::string battery_topic = this->get_parameter("battery_topic").as_string();
    std::string motor_topic = this->get_parameter("motor_topic").as_string();
    std::string altitude_topic = this->get_parameter("altitude_topic").as_string();
    std::string horscan_topic = this->get_parameter("hor_lidar_topic").as_string();
    std::string verscan_topic = this->get_parameter("vrt_lidar_topic").as_string();
    std::string target_topic = this->get_parameter("target_topic").as_string();
    std::string movement_topic = this->get_parameter("movement_topic").as_string();
    std::string arming_topic = this->get_parameter("arming_topic").as_string();

    _imu_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
        imu_topic, 10,
        std::bind(&Gravitational::_imu_callback, this, std::placeholders::_1));
    _gyro_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
        gyro_topic, 10,
        std::bind(&Gravitational::_gyro_callback, this, std::placeholders::_1));
    _battery_sub = this->create_subscription<sensor_msgs::msg::BatteryState>(
        battery_topic, 10,
        std::bind(&Gravitational::_batt_callback, this, std::placeholders::_1));
    _motor_sub = this->create_subscription<std_msgs::msg::Float32>(
        motor_topic, 10,
        std::bind(&Gravitational::_motor_callback, this, std::placeholders::_1));
    _altitude_sub = this->create_subscription<std_msgs::msg::Float32>(
        motor_topic, 10,
        std::bind(&Gravitational::_altitude_callback, this, std::placeholders::_1));
    _vertical_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        verscan_topic, 10,
        std::bind(&Gravitational::_vldr_callback, this, std::placeholders::_1));
    _horizontal_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        horscan_topic, 10,
        std::bind(&Gravitational::_hldr_callback, this, std::placeholders::_1));
    _target_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
        target_topic, 10,
        std::bind(&Gravitational::_target_callback, this, std::placeholders::_1));

    _movement_pub =
        this->create_publisher<geometry_msgs::msg::Quaternion>(movement_topic, 10);
    _arming_pub = this->create_publisher<std_msgs::msg::Bool>(arming_topic, 10);
    _pose_pub = this->create_publisher<visualization_msgs::msg::Marker>("/target", 10);

    _mtr_speed = 0;
    _drone_alt = 0;
    _ldr_alt = 0;

    ZERO_VEC(_g_targ);
    ZERO_VEC(_g_gyro);
    ZERO_VEC(_g_horizontal);
    ZERO_VEC(_g_vertical);
    ZERO_VEC(_g_realsense);
    ZERO_VEC(_g_last);

    _ctrl_timer = this->create_wall_timer(std::chrono::milliseconds(50),
                                          std::bind(&Gravitational::ctrl_callback, this));
}

void Gravitational::_imu_callback(const geometry_msgs::msg::Vector3& imu) {
    (void) (imu);
}
void Gravitational::_gyro_callback(const geometry_msgs::msg::Vector3& gyro) {
    _g_gyro = gyro;
}
void Gravitational::_batt_callback(const sensor_msgs::msg::BatteryState& batt) {
    (void) (batt);
}
void Gravitational::_motor_callback(const std_msgs::msg::Float32& mtr) {
    _mtr_speed = mtr.data;
}
void Gravitational::_altitude_callback(const std_msgs::msg::Float32& alt) {
    _drone_alt = alt.data;
}
void Gravitational::_vldr_callback(const sensor_msgs::msg::LaserScan& ldr) {
    ZERO_VEC(_g_horizontal)
    for (size_t i = 0; i < ldr.ranges.size(); i++) {
        float angle = ldr.angle_min + i * ldr.angle_increment;
        if (angle < M_PI) {
            _g_vertical.z += cos(angle) * ldr.ranges[i];
            _g_vertical.y += sin(angle) * ldr.ranges[i];
        } else {
            _g_vertical.z -= cos(angle) * ldr.ranges[i];
            _g_vertical.y -= sin(angle) * ldr.ranges[i];
        }
    }
}
void Gravitational::_hldr_callback(const sensor_msgs::msg::LaserScan& ldr) {
    ZERO_VEC(_g_horizontal);
    float aav = 0;
    for (size_t i = 0; i < ldr.ranges.size(); i++) {
        float angle = ldr.angle_min + i * ldr.angle_increment;
        aav += angle;
        if (angle < 0) {
            _g_horizontal.x += cos(angle) * (1/(ldr.ranges[i]+0.001)) * ldr.intensities[i]/255;
            _g_horizontal.y += sin(angle) * (1/(ldr.ranges[i]+0.001)) * ldr.intensities[i]/255;
        } else {
            _g_horizontal.x -= cos(angle) * (1/(ldr.ranges[i]+0.001)) * ldr.intensities[i]/255;
            _g_horizontal.y -= sin(angle) * (1/(ldr.ranges[i]+0.001)) * ldr.intensities[i]/255;
        }
    }
    _g_horizontal.x = (_g_horizontal.x / ldr.ranges.size());
    _g_horizontal.y = (_g_horizontal.y / ldr.ranges.size());
    RCLCPP_INFO(get_logger(), "angle average: %0.2f\n", aav/ldr.ranges.size());
}
void Gravitational::_target_callback(const geometry_msgs::msg::Vector3& target) {
    _g_targ = target;
}

void Gravitational::ctrl_callback(void) {
    // Always arm (for now)
    std_msgs::msg::Bool armed;
    armed.data = true;
    _arming_pub->publish(armed);

    // Map x/y [-100,100] -> roll/pitch in radians
    // e.g. ±20 → ±10 degrees tilt
    // float max_angle_rad = 180.0f * M_PI / 180.0f;              // 10 degrees
    // float roll = (_g_horizontal.y) * max_angle_rad;  // left/right tilt
    // float pitch = (_g_horizontal.x) * max_angle_rad; // forward/back tilt
    // float yaw = 0.0f;                                         // no yaw rotation for now

    // Convert roll/pitch/yaw -> quaternion
    // tf2::Quaternion q;
    // q.setRPY(roll, pitch, yaw);
    // q.normalize();
    
    // Fill pose message
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = this->now();
    m.ns = "vector";
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point start;
    start.x = 0;
    start.y = 0;
    start.x = 0;
    geometry_msgs::msg::Point end;
    end.x = _g_horizontal.x;
    end.y = _g_horizontal.y;
    end.z = 0.5;
    RCLCPP_INFO(get_logger(), "x: %0.2f y: %0.2f\n", _g_horizontal.x, _g_horizontal.y);
    m.points.push_back(start);
    m.points.push_back(end);

    // m.pose.position.x = 0.0;
    // m.pose.position.y = 0.0;
    // m.pose.position.z = 0.0;
    // m.pose.orientation.x = q.x();
    // m.pose.orientation.y = q.y();
    // m.pose.orientation.z = q.z();
    // m.pose.orientation.w = q.w();
    // Appearance
    m.scale.x = 0.02;  // shaft diameter
    m.scale.y = 0.05;  // head diameter
    m.scale.z = 0.05;  // head length

    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;

    _pose_pub->publish(m);

    // RCLCPP_INFO(this->get_logger(),
    //             "Roll: %.2f° Pitch: %.2f° -> q=(%.2f, %.2f, %.2f, %.2f)",
    //             roll * 180.0 / M_PI, pitch * 180.0 / M_PI, q.x(), q.y(), q.z(), q.w());
}
