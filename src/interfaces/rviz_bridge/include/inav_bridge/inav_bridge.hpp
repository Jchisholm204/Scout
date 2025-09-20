/**
 * @file inav_bridge.hpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2025-09-17
 * @modified Last Modified: 2025-09-17
 *
 * @copyright Copyright (c) 2025
 */

#ifndef _INAV_BRIDGE_HPP_
#define _INAV_BRIDGE_HPP_

#include "FlightController.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/vector3.h>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.h>
#include <tf2_ros/transform_broadcaster.h>

class INAVBridge : public rclcpp::Node {
  public:
    INAVBridge();
    ~INAVBridge();

  private:
    fcu::FlightController _fc;
    rclcpp::Duration _fall_time;
    int64_t _fall_speed;
    // Drone Communication Watchdog
    rclcpp::TimerBase::SharedPtr _wd_timer;
    void wd_callback(void);
    rclcpp::Time _wd_checkin;
    rclcpp::Duration _wd_maxtime;
    // Flight Controller Communication Callbacks
    void on_imu(const msp::msg::RawImu& imu);
    void on_motor(const msp::msg::Motor& mtr);
    void on_attitude(const msp::msg::Attitude& att);
    void on_altitude(const msp::msg::Altitude& alt);
    void on_analog(const msp::msg::Analog& alg);
    void on_debug_msg(const msp::msg::DebugMessage& dbg);
    // ROS Message Publishers
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr _imu_pub;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr _gyro_pub;
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr _orientation_pub;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr _battery_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _motor_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _altitude_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _rssi_pub;
    // ROS Message Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr _movement_sub;
    void movement_callback(const geometry_msgs::msg::Quaternion& qt);
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _arming_sub;
    void arming_callback(const std_msgs::msg::Bool& arm);
};

#endif
