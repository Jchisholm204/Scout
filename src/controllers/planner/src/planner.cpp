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

#include "planner/planner.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>

Planner::Planner() : Node("path_planner") {
    this->declare_parameter("imu_topic", "sim/imu");
    this->declare_parameter("battery_topic", "sim/batt");
    this->declare_parameter("mode_topic", "cb/mode");
    this->declare_parameter("wall_marker_topic", "/wall_markers");
    this->declare_parameter("open_marker_topic", "/open_markers");
    this->declare_parameter("position_topic", "sim/position");
    this->declare_parameter("velocity_topic", "sim/velocity");
    this->declare_parameter("movement_topic", "cb/vel_cmd");

    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    std::string battery_topic = this->get_parameter("battery_topic").as_string();
    std::string mode_topic = this->get_parameter("mode_topic").as_string();
    std::string wall_marker_topic = this->get_parameter("wall_marker_topic").as_string();
    std::string open_marker_topic = this->get_parameter("open_marker_topic").as_string();
    std::string position_topic = this->get_parameter("position_topic").as_string();
    std::string velocity_topic = this->get_parameter("velocity_topic").as_string();
    std::string movement_topic = this->get_parameter("movement_topic").as_string();

    // Create Subscriptions to drone topics
    _imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 10, std::bind(&Planner::_imu_callback, this, std::placeholders::_1));
    _battery_sub = this->create_subscription<sensor_msgs::msg::BatteryState>(
        battery_topic, 10,
        std::bind(&Planner::_batt_callback, this, std::placeholders::_1));
    _mode_sub = this->create_subscription<std_msgs::msg::UInt8>(
        mode_topic, 10, std::bind(&Planner::_mode_callback, this, std::placeholders::_1));

    // Subscriptions to Segmented LiDAR streams
    _wall_marker_sub = this->create_subscription<visualization_msgs::msg::Marker>(
        wall_marker_topic, 10,
        std::bind(&Planner::_wall_marker_callback, this, std::placeholders::_1));
    _open_marker_sub = this->create_subscription<visualization_msgs::msg::Marker>(
        open_marker_topic, 10,
        std::bind(&Planner::_open_marker_callback, this, std::placeholders::_1));

    _pos_sub = this->create_subscription<geometry_msgs::msg::Point>(
        position_topic, 10,
        std::bind(&Planner::_pos_callback, this, std::placeholders::_1));
    _vel_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
        velocity_topic, 10,
        std::bind(&Planner::_vel_callback, this, std::placeholders::_1));

    // Create Publishers
    _movement_pub =
        this->create_publisher<geometry_msgs::msg::Quaternion>(movement_topic, 10);

    // Setup operational timer
    _ctrl_timer = this->create_wall_timer(std::chrono::milliseconds(50),
                                          std::bind(&Planner::ctrl_callback, this));
}

Planner::~Planner() {
}

void Planner::_imu_callback(const sensor_msgs::msg::Imu& imu) {
    this->_imu = imu;
}

void Planner::_batt_callback(const sensor_msgs::msg::BatteryState& batt) {
    this->_battery = batt;
}

void Planner::_mode_callback(const std_msgs::msg::UInt8& mode) {
    this->_mode = (enum eCBMode) mode.data;
}

void Planner::_wall_marker_callback(const visualization_msgs::msg::Marker& msg) {
    _wall_markers = msg;
}

void Planner::_open_marker_callback(const visualization_msgs::msg::Marker& msg) {
    _open_markers = msg;
}

void Planner::_pos_callback(const geometry_msgs::msg::Point& position) {
    this->_position = position;
}

void Planner::_vel_callback(const geometry_msgs::msg::Vector3& velocity) {
    this->_velocity = velocity;
}

void Planner::ctrl_callback(void) {
    static int f_cmd = 0;
    geometry_msgs::msg::Quaternion cmd;
    // if (f_cmd == 0) {
    //     f_cmd = 1;
    //     cmd.x = 0.0;
    // } else if (f_cmd == 1) {
    //     f_cmd = -1;
    //     cmd.x = 0.06;
    // } else if (f_cmd == -1) {
    //     f_cmd = 0;
    //     cmd.x = -0.05;
    // }
    cmd.y = 0.00;
    // cmd.z = 0.00;
    // cmd.w = 0.00;
    if (_open_markers.points.size() >= 1) {
        cmd.x = 0.01 * _open_markers.points[0].y;
        cmd.w = -0.05 * _open_markers.points[0].x;
    }
    else{
        cmd.x = 0;
        cmd.w = 0;
    }
    RCLCPP_INFO(this->get_logger(), "Ctrl: %.3f %.3f %.3f %.3f", cmd.x, cmd.y, cmd.z,
                cmd.w);
    _movement_pub->publish(cmd);
}
