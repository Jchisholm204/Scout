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
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <unistd.h>
#include <cmath>

Planner::Planner() : Node("path_planner") {
    _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    this->declare_parameter("imu_topic", "/sim/imu");
    this->declare_parameter("battery_topic", "/sim/batt");
    this->declare_parameter("mode_topic", "/cb/mode");
    this->declare_parameter("wall_marker_topic", "/wall_markers");
    this->declare_parameter("open_marker_topic", "/open_markers");
    this->declare_parameter("position_topic", "/sim/position");
    this->declare_parameter("velocity_topic", "/sim/velocity");
    this->declare_parameter("movement_topic", "/cb/vel_cmd");
    this->declare_parameter("waypoint_topic", "/planner/current_waypoint");
    this->declare_parameter("drone_frame", "base_footprint");
    this->declare_parameter("world_frame", "world");
    this->declare_parameter("startup_delay_ms", 4000);
    this->declare_parameter("imu_transform_topic", "/planner/imu_orientation");
    this->declare_parameter("dcs_waypoint_topic", "/planner/dcs_waypoints");
    // this->declare_parameter("slam_topic", "/sim/position"); //change to actual SLAM topic when available

    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    std::string battery_topic = this->get_parameter("battery_topic").as_string();
    std::string mode_topic = this->get_parameter("mode_topic").as_string();
    std::string wall_marker_topic = this->get_parameter("wall_marker_topic").as_string();
    std::string open_marker_topic = this->get_parameter("open_marker_topic").as_string();
    std::string position_topic = this->get_parameter("position_topic").as_string();
    std::string velocity_topic = this->get_parameter("velocity_topic").as_string();
    std::string movement_topic = this->get_parameter("movement_topic").as_string();
    std::string waypoint_topic = this->get_parameter("waypoint_topic").as_string();
    this->drone_frame = this->get_parameter("drone_frame").as_string();
    this->world_frame = this->get_parameter("world_frame").as_string();
    int startup_delay_ms = this->get_parameter("startup_delay_ms").as_int();
    std::string imu_transform_topic = this->get_parameter("imu_transform_topic").as_string();
    std::string dcs_waypoint_topic = this->get_parameter("dcs_waypoint_topic").as_string();
    // std::string slam_topic = this->get_parameter("slam_topic").as_string();

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

    // //Subscription to SLAM localization data (position + orientation)
    // _slam_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    //     slam_topic, 10,
    //     std::bind(&Planner::_slam_callback, this, std::placeholders::_1)
    // );

    // Create Publishers
    _movement_pub =
        this->create_publisher<geometry_msgs::msg::Quaternion>(movement_topic, 10);
    _waypoint_pub = this->create_publisher<geometry_msgs::msg::PointStamped>(waypoint_topic, 10);

    _imu_transform_pub =
        this->create_publisher<geometry_msgs::msg::TransformStamped>(imu_transform_topic, 10);
    _dcs_waypoint_pub = 
        this->create_publisher<visualization_msgs::msg::Marker>(dcs_waypoint_topic, 10);
    // Deferred init: wait for wall markers, open markers, and position data to arrive
    _init_timer = this->create_wall_timer(
        std::chrono::milliseconds(startup_delay_ms),
        [this]() {
            _init_waypoint_callback();
        },
        nullptr  // callback group
    );

    // Setup operational timer
    _ctrl_timer = this->create_wall_timer(std::chrono::milliseconds(50),
                                          std::bind(&Planner::ctrl_callback, this));

    // Initialize orientation to identity to avoid math errors before SLAM starts
    _orientation.w = 1.0;
}

Planner::~Planner() {
}

void Planner::_init_waypoint_callback(void) {
    _init_timer->cancel();
    _init_timer.reset();
    update_waypoint();
}

void Planner::_imu_callback(const sensor_msgs::msg::Imu& imu) { 
    this->_orientation = imu.orientation;
    this->_imu = imu;

    if (!_position.has_value()) return;

    // 3. Update the TF Broadcast
    geometry_msgs::msg::TransformStamped transform;
    transform.header = imu.header;
    transform.header.frame_id = world_frame; // This is now your WCS world
    transform.child_frame_id = drone_frame;
    
    transform.transform.translation.x = _position.value().x;
    transform.transform.translation.y = _position.value().y;
    transform.transform.translation.z = _position.value().z;

    // IMPORTANT: Use the transformed orientation, not the raw sim orientation!
    transform.transform.rotation = _orientation;
    
    _tf_broadcaster->sendTransform(transform);
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
    if (!_position.has_value()) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for position data to process markers...");
        return; 
    }

    _open_markers = msg;
    waypoints_W.clear();

    // ADD THIS: Initialize the DCS visualization marker
    visualization_msgs::msg::Marker dcs_marker;
    dcs_marker.header.frame_id = drone_frame; // Keep it in the drone frame
    dcs_marker.header.stamp = this->now();
    dcs_marker.ns = "dcs_waypoints";
    dcs_marker.id = 0;
    dcs_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST; // Spheres are easy to see in RViz
    dcs_marker.action = visualization_msgs::msg::Marker::ADD;
    dcs_marker.scale.x = 1; // Adjust size as needed
    dcs_marker.scale.y = 1;
    dcs_marker.scale.z = 1;
    dcs_marker.color.r = 0.0f;
    dcs_marker.color.g = 0.5f;
    dcs_marker.color.b = 1.0f; // Make them blue to distinguish from your green open spaces
    dcs_marker.color.a = 1.0f;

    // Assuming markers are LINE_LIST where every 2 points define a gate/gap
    if (msg.type == visualization_msgs::msg::Marker::LINE_LIST) {
        for (size_t i = 0; i + 1 < msg.points.size(); i += 2) {
            geometry_msgs::msg::Point mid;
            mid.x = (msg.points[i].x + msg.points[i+1].x) / 2.0;
            mid.y = (msg.points[i].y + msg.points[i+1].y) / 2.0; 
            mid.z = (msg.points[i].z + msg.points[i+1].z) / 2.0; 

            // ADD THIS: Push the raw DCS midpoint to the visualization marker
            dcs_marker.points.push_back(mid);

            mid = this->convert_DCS_to_WCS(mid, _position.value(), _orientation); //convert to WCS
            waypoints_W.push_back(mid);
        }
    }
    
    // ADD THIS: Publish the DCS marker
    _dcs_waypoint_pub->publish(dcs_marker);
}

void Planner::_pos_callback(const geometry_msgs::msg::Point& position) {
    if (!_position.has_value()) {
        _position = geometry_msgs::msg::Point(); 
    }
    
    this->_position = position;
}

void Planner::_vel_callback(const geometry_msgs::msg::Vector3& velocity) { //Not used anywhere
    this->_velocity = velocity;
}

// void Planner::_slam_callback(const geometry_msgs::msg::PoseStamped& msg) {
//     this->_position_abs = msg.pose.position;
//     this->_orientation_abs = msg.pose.orientation;
// }

geometry_msgs::msg::Point Planner::convert_DCS_to_WCS(const geometry_msgs::msg::Point& waypoint_DCS, const geometry_msgs::msg::Point& position, const geometry_msgs::msg::Quaternion& orientation)
{
    // 1. Convert the orientation message to a tf2::Quaternion
    tf2::Quaternion q_orig;
    tf2::fromMsg(orientation, q_orig);
    
    // (Optional debugging: you can still extract and print the yaw if needed)
    // double roll, pitch, yaw;
    // tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw); 
    // printf("Yaw: %f\n", yaw);

    // 2. Initialize Transform directly with the full 3D rotation and translation
    tf2::Transform transform_W_D(
        q_orig, 
        tf2::Vector3(position.x, position.y, position.z)
    );

    // 3. Apply Transform to the waypoint
    tf2::Vector3 waypoint_WCS = transform_W_D * tf2::Vector3(waypoint_DCS.x, waypoint_DCS.y, waypoint_DCS.z);
    
    // 4. Construct and return output
    geometry_msgs::msg::Point p;
    p.x = waypoint_WCS.x();
    p.y = waypoint_WCS.y();
    p.z = waypoint_WCS.z();
    
    return p;
}

void Planner::update_waypoint(void) {
    printf("Updating Waypoint\n Number of midpoints: %zu\n", waypoints_W.size());
    
    if ((!waypoints_W.empty()) && _position.has_value()) { //first time this fails
        //convert to WCS and add to tree
        for (const auto& waypoint_W : waypoints_W){
            printf("Updating Waypoint W: %f %f %f\n", waypoint_W.x, waypoint_W.y, waypoint_W.z);
            if( waypoint_W.x > _position.value().x ) {
                //update waypoint
                current_waypoint.header.frame_id = world_frame;
                current_waypoint.header.stamp = this->now();
                current_waypoint.point = waypoint_W;
                // For now, just take the first one that satisfies the condition
                break;
            }
        }
    }
} 

void Planner::ctrl_callback(void) {

    geometry_msgs::msg::Quaternion cmd;

    if (!_position.has_value()) {
        RCLCPP_WARN(this->get_logger(), "No position data yet, cannot control.");
        return;
    }

    // 1. Check waypoint distance (X-Y plane only)
    double dx = current_waypoint.point.x - _position.value().x;
    double dy = current_waypoint.point.y - _position.value().y;
    double dist = std::sqrt(dx*dx + dy*dy);
    
    if(dist < 2){ // Threshold
        update_waypoint();
        // Recalculate dx, dy, and dist for the new waypoint
        dx = current_waypoint.point.x - _position.value().x;
        dy = current_waypoint.point.y - _position.value().y;
        dist = std::sqrt(dx*dx + dy*dy);
    }
    cmd.y = 0.00;

    printf("Current Waypoint: %f %f %f\n", current_waypoint.point.x, current_waypoint.point.y, current_waypoint.point.z);
    printf("Current Position: %f %f %f\n", _position.value().x, _position.value().y, _position.value().z);

    // Check if current_waypoint is valid
    if (std::abs(current_waypoint.point.x) > 0.001 && std::abs(current_waypoint.point.y) > 0.001) { 
        
        // --- DISTANCE ERROR (Magnitude) ---
        // 'dist' is exactly the magnitude of the distance in the X-Z plane
        double distance_error = dist;

        // --- HEADING ERROR (Rotation) ---
        // 1. Calculate intended global heading towards waypoint in X-Z plane
        double target_heading = std::atan2(dy, dx);
        printf("Target Heading: %f\n", target_heading);

        // 2. Get current global heading based on the DCS Y-axis (Forward)
        tf2::Quaternion rot_W;
        tf2::fromMsg(_orientation, rot_W);
        
        // Define the forward direction in your DCS (the Y-axis)
        tf2::Vector3 forward_DCS(0.0, 1.0, 0.0);

        // Rotate this forward vector into the World Coordinate System
        tf2::Vector3 forward_WCS = tf2::quatRotate(rot_W, forward_DCS);

        // Calculate the heading of this forward vector in the global X-Y plane
        double current_heading = std::atan2(forward_WCS.y(), forward_WCS.x());
        printf("Current Heading: %f\n", current_heading);

        // 3. Calculate heading error
        double heading_error = target_heading - current_heading;

        // 4. Normalize the angle to be strictly between -PI and PI
        // This prevents the drone from spinning the long way around
        while (heading_error > M_PI)  heading_error -= 2.0 * M_PI;
        while (heading_error < -M_PI) heading_error += 2.0 * M_PI;


        cmd.x = .005 * distance_error; // Forward tilt based on distance
        cmd.w = -.7 * heading_error;   // Spin based on heading difference
        cmd.z = 0.1; // Kept from your original code
        
        // --- LIMITERS ---
        if(cmd.x > 0.075) cmd.x = 0.075;
        // Since distance is always positive, cmd.x won't be negative here 
        // unless you want the drone to tilt backwards if it overshoots.
        
        if(cmd.w > 0.2) cmd.w = 0.2;
        if(cmd.w < -0.2) cmd.w = -0.2;

    } else {
        cmd.x = 0;
        cmd.w = 0;
        cmd.z = 0;
    }

    RCLCPP_INFO(this->get_logger(), "Ctrl: %.3f %.3f %.3f %.3f", cmd.x, cmd.y, cmd.z, cmd.w);
    
    _movement_pub->publish(cmd);
    
    current_waypoint.header.frame_id = world_frame;
    current_waypoint.header.stamp = this->now();
    _waypoint_pub->publish(current_waypoint);
}
