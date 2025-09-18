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
    _fc.setLoggingLevel(msp::client::LoggingLevel::SILENT);
    _fc.connect("/dev/ttyACM0", 115200);
    _timer =
    this->create_wall_timer(std::chrono::milliseconds(50),
                            std::bind(&INAVBridge::timer_callback, this));

    _fc.subscribe<msp::msg::RawImu>(std::bind(&INAVBridge::imu_callback, this, std::placeholders::_1), 0.1);
}

void INAVBridge::timer_callback(void) {
}

void INAVBridge::imu_callback(const msp::msg::RawImu& imu){
    auto im = msp::msg::ImuSI(imu, 512.0, 1.0 / 4.096, 0.92f / 10.0f, 9.80665f);
    RCLCPP_INFO(this->get_logger(), "ACC: %0.2f %0.2f %0.2f\n", im.acc[0](), im.acc[1](), im.acc[2]());
    RCLCPP_INFO(this->get_logger(), "GYRO: %0.2f %0.2f %0.2f\n", im.gyro[0](), im.gyro[1](), im.gyro[2]());
    std::cout << msp::msg::ImuSI(imu, 512.0, 1.0 / 4.096, 0.92f / 10.0f, 9.80665f);
}
