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

#include "inav_bridge/inav_bridge.hpp"
#include "FlightController.hpp"

INAVBridge::INAVBridge() : Node("inav_bridge") {
    // Setup MSP Link
    _fc.setLoggingLevel(msp::client::LoggingLevel::SILENT);
    _fc.connect("/dev/ttyACM0", 115200);

    // Subscriptions to MSP topics
    _fc.subscribe<msp::msg::RawImu>(std::bind(&INAVBridge::imu_callback, this, std::placeholders::_1), 0.1);
    // Setup MCU flight mode
    fcu::FlightMode mode;
    mode.primary = fcu::FlightMode::PRIMARY_MODE::ANGLE;
    mode.secondary = fcu::FlightMode::SECONDARY_MODE::NAV_ALTHOLD;
    mode.modifier = fcu::FlightMode::MODIFIER::ARM;
    _fc.setFlightMode(mode);
    // Create timers
    _timer =
        this->create_wall_timer(std::chrono::milliseconds(50),
                            std::bind(&INAVBridge::timer_callback, this));
    // Create TF2 Transform Broadcaster
    _timer_tfbcast = 
        this->create_wall_timer(std::chrono::milliseconds(50),
                            std::bind(&INAVBridge::tfbcast_callback, this));
    _tfbcast = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
}

void INAVBridge::timer_callback(void) {
}

void INAVBridge::imu_callback(const msp::msg::RawImu& imu){
    _imu = std::make_shared<msp::msg::ImuSI>(imu, 512.0, 1.0 / 4.096, 0.92f / 10.0f, 9.80665f);
    RCLCPP_INFO(this->get_logger(), "ACC: %0.2f %0.2f %0.2f\n", _imu->acc[0](), _imu->acc[1](), _imu->acc[2]());
    RCLCPP_INFO(this->get_logger(), "GYRO: %0.2f %0.2f %0.2f\n", _imu->gyro[0](), _imu->gyro[1](), _imu->gyro[2]());
    std::cout << msp::msg::ImuSI(imu, 512.0, 1.0 / 4.096, 0.92f / 10.0f, 9.80665f);
}

void INAVBridge::tfbcast_callback(void){
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_footprint";

    // Example position
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 1;

    // --- Compute orientation from accelerometer ---
    float ax = _imu->acc[0]();
    float ay = _imu->acc[1]();
    float az = _imu->acc[2]();

    // Normalize acceleration vector
    float norm = std::sqrt(ax * ax + ay * ay + az * az);
    if (norm > 1e-6f) {
        ax /= norm;
        ay /= norm;
        az /= norm;
    }

    // Compute roll and pitch from gravity vector
    float roll  = std::atan2(ay, az);                    // rotation around x-axis
    float pitch = std::atan2(-ax, std::sqrt(ay*ay + az*az));  // rotation around y-axis

    // Convert roll/pitch to quaternion (yaw = 0)
    float cy = std::cos(0.0f * 0.5f);  // yaw = 0
    float sy = std::sin(0.0f * 0.5f);
    float cp = std::cos(pitch * 0.5f);
    float sp = std::sin(pitch * 0.5f);
    float cr = std::cos(roll * 0.5f);
    float sr = std::sin(roll * 0.5f);

    t.transform.rotation.w = cr * cp * cy + sr * sp * sy;
    t.transform.rotation.x = sr * cp * cy - cr * sp * sy;
    t.transform.rotation.y = cr * sp * cy + sr * cp * sy;
    t.transform.rotation.z = cr * cp * sy - sr * sp * cy;

    _tfbcast->sendTransform(t);
}

