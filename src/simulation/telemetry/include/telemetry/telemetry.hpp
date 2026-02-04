/**
 * @file telemetry.hpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief 
 * @version 0.1
 * @date Created: 2026-02-03
 * @modified Last Modified: 2026-02-03
 *
 * @copyright Copyright (c) 2026
 */


#ifndef _TELEMETRY_HPP_
#define _TELEMETRY_HPP_

#define TELEM_PORT 9001

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/float32.hpp>

#pragma pack(push, 1) // Critical for network packet alignment
typedef struct {
    float timestamp;              // 4 bytes
    float posX, posY, posZ;       // 12 bytes
    float rotW, rotX, rotY, rotZ; // 16 bytes (Quaternions)
    float velX, velY, velZ;       // 12 bytes
    float gyroP, gyroR, gyroY;    // 12 bytes
    float inT, inY, inP, inR;     // 16 bytes
    float batPct, batVolt;        // 8 bytes
    uint8_t motorCount;           // 1 byte
    float motors[4];              // 16 bytes (for a quad)
} lf_telemetry_packet_t;
#pragma pack(pop)

class Telemetry : public rclcpp::Node {
    public:
        Telemetry();
        ~Telemetry();
    private:
        rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr _batt_pub;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr _imu_pub;
        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr _pos_pub;
        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr _vel_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _time_pub;
        rclcpp::TimerBase::SharedPtr _callback_timer;
        int _udp_fp = 0;
        void _callback(void);
};

#endif
