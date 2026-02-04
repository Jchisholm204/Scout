/**
 * @file localization.hpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief 
 * @version 0.1
 * @date Created: 2026-02-04
 * @modified Last Modified: 2026-02-04
 *
 * @copyright Copyright (c) 2026
 */


#ifndef _LOCALIZATION_HPP_
#define _LOCALIZATION_HPP_

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <geometry_msgs/msg/vector3.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <slg_msgs/msg/segment_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

class Localization: public rclcpp::Node {
  public:
    Localization();
    ~Localization();

  private:
    // LiDAR data Subscriptions (either from sim or USB IF)
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _vertical_scan_sub;
    void _vldr_callback(const sensor_msgs::msg::LaserScan& ldr);
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _horizontal_scan_sub;
    void _hldr_callback(const sensor_msgs::msg::LaserScan& ldr);

    // LiDAR segmentation subscription
    rclcpp::Subscription<slg_msgs::msg::SegmentArray>::SharedPtr _hldr_seg_sub;
    void _hldr_seg_callback(const slg_msgs::msg::SegmentArray& seg_msg);

    // Visualization Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _wall_marker_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _open_marker_pub;
};

#endif
