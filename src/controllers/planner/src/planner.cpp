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
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <unistd.h>
#include <cmath>

Planner::Planner() : Node("path_planner") {
    this->declare_parameter("imu_topic", "sim/imu");
    this->declare_parameter("battery_topic", "sim/batt");
    this->declare_parameter("mode_topic", "cb/mode");
    this->declare_parameter("wall_marker_topic", "/wall_markers");
    this->declare_parameter("open_marker_topic", "/open_markers");
    this->declare_parameter("position_topic", "sim/position");
    this->declare_parameter("velocity_topic", "sim/velocity");
    this->declare_parameter("movement_topic", "cb/vel_cmd");
    this->declare_parameter("slam_topic", "sim/pose");

    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    std::string battery_topic = this->get_parameter("battery_topic").as_string();
    std::string mode_topic = this->get_parameter("mode_topic").as_string();
    std::string wall_marker_topic = this->get_parameter("wall_marker_topic").as_string();
    std::string open_marker_topic = this->get_parameter("open_marker_topic").as_string();
    std::string position_topic = this->get_parameter("position_topic").as_string();
    std::string velocity_topic = this->get_parameter("velocity_topic").as_string();
    std::string movement_topic = this->get_parameter("movement_topic").as_string();
    std::string slam_topic = this->get_parameter("slam_topic").as_string();

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

    //Subscription to SLAM localization data (position + orientation)
    _slam_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        slam_topic, 10,
        std::bind(&Planner::_slam_callback, this, std::placeholders::_1)
    );

    // Create Publishers
    _movement_pub =
        this->create_publisher<geometry_msgs::msg::Quaternion>(movement_topic, 10);

    // Init first waypoint - assumes we have SLAM data and LIDAR data - TODO timing coordination??
    // this->update_waypoint();

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
    //convert to absolute coords
    _wall_markers = msg;
}

void Planner::_open_marker_callback(const visualization_msgs::msg::Marker& msg) {
    _open_markers = msg;
    latest_midpoints_D.clear();
    // Assuming markers are LINE_LIST where every 2 points define a gate/gap
    if (msg.type == visualization_msgs::msg::Marker::LINE_LIST) {
        for (size_t i = 0; i + 1 < msg.points.size(); i += 2) {
            geometry_msgs::msg::Point mid;
            mid.x = (msg.points[i].x + msg.points[i+1].x) / 2.0;
            mid.y = (msg.points[i].y + msg.points[i+1].y) / 2.0;
            mid.z = (msg.points[i].z + msg.points[i+1].z) / 2.0; //this is prolly 0
            latest_midpoints_D.push_back(mid);
        }
    }
}

void Planner::_pos_callback(const geometry_msgs::msg::Point& position) {
    this->_position = position;
}

void Planner::_vel_callback(const geometry_msgs::msg::Vector3& velocity) {
    this->_velocity = velocity;
}

void Planner::_slam_callback(const geometry_msgs::msg::PoseStamped& msg) {
    this->_position_abs = msg.pose.position;
    this->_orientation_abs = msg.pose.orientation;
}

geometry_msgs::msg::Point Planner::convert_DCS_to_WCS(geometry_msgs::msg::Point waypoint_DCS, geometry_msgs::msg::Point position, geometry_msgs::msg::Quaternion orientation){
    tf2::Vector3 translation(position.x, position.y, position.z);
    tf2::Quaternion rotation;
    tf2::fromMsg(orientation, rotation);

    tf2::Transform transform_W_D;
    transform_W_D.setOrigin(translation);
    transform_W_D.setRotation(rotation);

    //assuming x,y,0
    tf2::Vector3 waypoint(waypoint_DCS.x, waypoint_DCS.y, waypoint_DCS.z);

    //assuming order is correct for waypoint
    tf2::Vector3 waypoint_WCS = transform_W_D * waypoint;
    
    geometry_msgs::msg::Point p;
    p.x = waypoint_WCS.x();
    p.y = waypoint_WCS.y();
    p.z = waypoint_WCS.z();
    return p;
}

void Planner::update_waypoint(void) {
    
    if (!latest_midpoints_D.empty()) { //critical condition else downstream logic breaks
        //convert to WCS and add to tree
        for (const auto& marker_midpoint_D : latest_midpoints_D){
            geometry_msgs::msg::Point waypoint_W = convert_DCS_to_WCS(marker_midpoint_D, _position_abs, _orientation_abs);
            if( waypoint_W.x > _position_abs.x ) { //CHECK THAT IT'S THE X COORD - DEPENDS ON WCS ORIENTATION IN SPACE
                //add elements to waypoints tree "waypoints"
                // waypoints.add(waypoint_W)
                
                //update waypoint
                current_waypoint = waypoint_W;
                // For now, just take the first one that satisfies the condition
                break;
            }
        }
    }
} 

void Planner::ctrl_callback(void) {
    // static int f_cmd = 0;
    geometry_msgs::msg::Quaternion cmd;

    //check waypoint distance
    double dist = std::sqrt(std::pow(current_waypoint.x - _position_abs.x, 2) + std::pow(current_waypoint.y - _position_abs.y, 2));
    if(dist < 0.5){ // Threshold
        update_waypoint();
    }

    cmd.y = 0.00;
    // Check if current_waypoint is valid (not 0,0,0) - simple check
    if (std::abs(current_waypoint.x) > 0.001 || std::abs(current_waypoint.y) > 0.001) {
        //worry about controls later
        //control towards waypoint - set quaternion x for forwards, quaternion w for rotate side to side
        
        // Transform waypoint to body frame to get error
        tf2::Vector3 pos_W(_position_abs.x, _position_abs.y, _position_abs.z);
        tf2::Quaternion rot_W;
        tf2::fromMsg(_orientation_abs, rot_W);
        tf2::Transform T_WD(rot_W, pos_W);
        
        tf2::Vector3 wp_W(current_waypoint.x, current_waypoint.y, current_waypoint.z);
        tf2::Vector3 err_W = wp_W - pos_W;
        tf2::Vector3 err_D = T_WD.inverse() * err_W;

        cmd.x = 0.5 * err_D.x(); // Forward P control
        cmd.w = 1.0 * err_D.y(); // Yaw P control (turn towards waypoint)
        
        // Limiters
        if(cmd.x > 0.5) cmd.x = 0.5;
        if(cmd.w > 0.5) cmd.w = 0.5;
        if(cmd.w < -0.5) cmd.w = -0.5;

    } else {
        cmd.x = 0;
        cmd.w = 0;
    }
    RCLCPP_INFO(this->get_logger(), "Ctrl: %.3f %.3f %.3f %.3f", cmd.x, cmd.y, cmd.z,
                cmd.w);
    _movement_pub->publish(cmd);
}
