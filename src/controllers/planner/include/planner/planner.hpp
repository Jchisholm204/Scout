/**
 * @file planner.hpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2026-02-03
 * @modified Last Modified: 2026-02-03
 *
 * @copyright Copyright (c) 2026
 */

#ifndef _PLANNER_HPP_
#define _PLANNER_HPP_

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/vector3.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

class Planner : public rclcpp::Node {
  public:
    Planner();
    ~Planner();

  private:
    // Drone Data Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
    void _imu_callback(const sensor_msgs::msg::Imu& imu);
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr _battery_sub;
    void _batt_callback(const sensor_msgs::msg::BatteryState& batt);
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr _mode_sub;
    void _mode_callback(const std_msgs::msg::UInt8& mode);

    // LiDAR data Subscriptions (either from sim or USB IF)
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _vertical_scan_sub;
    void _vldr_callback(const sensor_msgs::msg::LaserScan& ldr);
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _horizontal_scan_sub;
    void _hldr_callback(const sensor_msgs::msg::LaserScan& ldr);

    // Positioning Subscriptions (either from sim or SLAM)
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr _pos_sub;
    void _pos_callback(const geometry_msgs::msg::Point& position);
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _vel_sub;
    void _vel_callback(const geometry_msgs::msg::Vector3& velocity);

    // ROS Message Publishers
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr _movement_pub;

    // Runtime
    rclcpp::TimerBase::SharedPtr _ctrl_timer;
    void ctrl_callback(void);

    sensor_msgs::msg::Imu _imu;
    sensor_msgs::msg::BatteryState _battery;
    enum class eCBMode {
        eModeDisabled,
        eModeInit,
        eModeRC,
        eModeRCAuto,
        eModeAuto,
        eModeStalled,
        eModeFault
    } _mode;
    sensor_msgs::msg::LaserScan _front_lidar;
    sensor_msgs::msg::LaserScan _vertical_lidar;
    geometry_msgs::msg::Point _position;
    geometry_msgs::msg::Vector3 _velocity;
};

#endif
