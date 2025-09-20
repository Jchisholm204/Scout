/**
 * @file gravitational.hpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.2
 * @date Created: 2025-09-17
 * @modified Last Modified: 2025-09-20
 *
 * @copyright Copyright (c) 2025
 */

#ifndef _GRAVITATIONAL_HPP_
#define _GRAVITATIONAL_HPP_

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/vector3.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

class Gravitational : public rclcpp::Node {
  public:
    Gravitational();

  private:
    // ROS Message Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _imu_sub;
    void _imu_callback(const geometry_msgs::msg::Vector3& imu);
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _gyro_sub;
    void _gyro_callback(const geometry_msgs::msg::Vector3& gyro);
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr _battery_sub;
    void _batt_callback(const sensor_msgs::msg::BatteryState& batt);
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _motor_sub;
    void _motor_callback(const std_msgs::msg::Float32& mtr);
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _altitude_sub;
    void _altitude_callback(const std_msgs::msg::Float32& alt);
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _vertical_scan_sub;
    void _vldr_callback(const sensor_msgs::msg::LaserScan& ldr);
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _horizontal_scan_sub;
    void _hldr_callback(const sensor_msgs::msg::LaserScan& ldr);
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _target_sub;
    void _target_callback(const geometry_msgs::msg::Vector3& target);
    // ROS Message Publishers
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr _movement_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _pose_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _arming_pub;

    // Saved Sensor Data
    float _mtr_speed;
    float _drone_alt;
    float _ldr_alt;

    // Gravitational points
    geometry_msgs::msg::Vector3 _g_targ;
    geometry_msgs::msg::Vector3 _g_gyro;
    geometry_msgs::msg::Vector3 _g_horizontal;
    geometry_msgs::msg::Vector3 _g_vertical;
    geometry_msgs::msg::Vector3 _g_realsense;
    geometry_msgs::msg::Vector3 _g_last;

    // Runtime
    rclcpp::TimerBase::SharedPtr _ctrl_timer;
    void ctrl_callback(void);
};

#endif
