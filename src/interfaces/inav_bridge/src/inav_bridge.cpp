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
    auto im = msp::msg::ImuSI(imu, 512.0, 1.0 / 4.096, 0.92f / 10.0f, 9.80665f);
    RCLCPP_INFO(this->get_logger(), "ACC: %0.2f %0.2f %0.2f\n", im.acc[0](), im.acc[1](), im.acc[2]());
    RCLCPP_INFO(this->get_logger(), "GYRO: %0.2f %0.2f %0.2f\n", im.gyro[0](), im.gyro[1](), im.gyro[2]());
    std::cout << msp::msg::ImuSI(imu, 512.0, 1.0 / 4.096, 0.92f / 10.0f, 9.80665f);
}

void INAVBridge::tfbcast_callback(void){
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "map";
    t.child_frame_id = "sjtu_drone";
    t.child_frame_id = "base_footprint";
    // Set drone position (example values, update from sensors or control)
    t.transform.translation.x = 1.0;
    t.transform.translation.y = 2.0;
    t.transform.translation.z = 0.5;

    // Set drone orientation (quaternion)
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    _tfbcast->sendTransform(t);
}
