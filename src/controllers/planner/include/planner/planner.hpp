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
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
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
#include <optional>
#include <vector>

//#include <nanoflann.hpp>


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

    // LiDAR segmentation subscription
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr _wall_marker_sub;
    void _wall_marker_callback(const visualization_msgs::msg::Marker& msg);
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr _open_marker_sub;
    void _open_marker_callback(const visualization_msgs::msg::Marker& msg);

    // Positioning Subscriptions (either from sim or SLAM)
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr _pos_sub;
    void _pos_callback(const geometry_msgs::msg::Point& position);
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _vel_sub;
    void _vel_callback(const geometry_msgs::msg::Vector3& velocity);

    // SLAM Subscription
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _slam_sub;
    void _slam_callback(const geometry_msgs::msg::PoseStamped& msg);

    // ROS Message Publishers
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr _movement_pub;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr _waypoint_pub;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr _imu_transform_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr _dcs_waypoint_pub;

    // Runtime
    rclcpp::TimerBase::SharedPtr _ctrl_timer;
    void ctrl_callback(void);
    rclcpp::TimerBase::SharedPtr _init_timer;
    void _init_waypoint_callback(void);

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
    visualization_msgs::msg::Marker _wall_markers;
    visualization_msgs::msg::Marker _open_markers;
    // geometry_msgs::msg::Point _position;
    std::optional<geometry_msgs::msg::Point> _position;

    geometry_msgs::msg::Vector3 _velocity;
    geometry_msgs::msg::Quaternion _orientation;
    geometry_msgs::msg::Point _position_abs;
    geometry_msgs::msg::Quaternion _orientation_abs;

    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;

    //init nanoflann kd tree "waypoints", which holds 3D points
    // nanoflann::tree waypoints;
    std::vector<geometry_msgs::msg::Point> waypoints_W;

    geometry_msgs::msg::PointStamped current_waypoint;
    std::string world_frame;
    std::string drone_frame;

    geometry_msgs::msg::Point convert_DCS_to_WCS(
      const geometry_msgs::msg::Point& waypoint_DCS, 
      const geometry_msgs::msg::Point& position, 
      const geometry_msgs::msg::Quaternion& orientation
  );    
    void update_waypoint(void);

};

#endif
