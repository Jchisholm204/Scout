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
    _orientation_abs.w = 1.0;
}

Planner::~Planner() {
}

void Planner::_init_waypoint_callback(void) {
    _init_timer->cancel();
    _init_timer.reset();
    update_waypoint();
}

void Planner::_imu_callback(const sensor_msgs::msg::Imu& imu) {
    tf2::Quaternion q_sim;
    tf2::fromMsg(imu.orientation, q_sim);

    // 1. Define the Basis Transformation Matrix
    // Mapping: WCS_x = Sim -y | WCS_y = Sim -z | WCS_z = Sim x
    tf2::Matrix3x3 m_basis(
         0, -1,  0,  // New X-axis is Sim -Y
         0,  0, -1,  // New Y-axis is Sim -Z
         1,  0,  0   // New Z-axis is Sim X
    );
    
    // 2. Transform the orientation into the new world frame
    tf2::Matrix3x3 m_sim(q_sim);
    tf2::Matrix3x3 m_logic = m_basis * m_sim * m_basis.transpose();
    
    tf2::Quaternion q_logic;
    m_logic.getRotation(q_logic);
    _orientation_abs = tf2::toMsg(q_logic); // This is what your control logic uses
    
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
    transform.transform.rotation = _orientation_abs; 
    
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
    // Assuming markers are LINE_LIST where every 2 points define a gate/gap LOOK AT PARSING
    if (msg.type == visualization_msgs::msg::Marker::LINE_LIST) {
        for (size_t i = 0; i + 1 < msg.points.size(); i += 2) {
            geometry_msgs::msg::Point mid;
            mid.x = (msg.points[i].x + msg.points[i+1].x) / 2.0;
            mid.y = (msg.points[i].y + msg.points[i+1].y) / 2.0; //this is prolly 0
            mid.z = (msg.points[i].z + msg.points[i+1].z) / 2.0; 
            printf("Midpoint: %f %f %f\n", mid.x, mid.y, mid.z);

            mid = this->convert_DCS_to_WCS(mid, _position.value(), _orientation_abs); //convert to WCS
            printf("Midpoint WCS: %f %f %f\n", mid.x, mid.y, mid.z);
            waypoints_W.push_back(mid);
        }
    }
}

void Planner::_pos_callback(const geometry_msgs::msg::Point& position) {
    if (!_position.has_value()) {
        _position = geometry_msgs::msg::Point(); 
    }
    
    this->_position->x = -position.y;
    this->_position->y = -position.z;
    this->_position->z = position.x;
}

void Planner::_vel_callback(const geometry_msgs::msg::Vector3& velocity) { //Not used anywhere
    this->_velocity = velocity;
}

// void Planner::_slam_callback(const geometry_msgs::msg::PoseStamped& msg) {
//     this->_position_abs = msg.pose.position;
//     this->_orientation_abs = msg.pose.orientation;
// }

// only rotates around y-axis, not x or z
geometry_msgs::msg::Point Planner::convert_DCS_to_WCS( geometry_msgs::msg::Point waypoint_DCS, geometry_msgs::msg::Point position, geometry_msgs::msg::Quaternion orientation)
{
    // 1. Setup Translation
    tf2::Vector3 translation(position.x, position.y, position.z);

    // 2. Isolate Y-axis Rotation
    tf2::Quaternion original_rotation;
    tf2::fromMsg(orientation, original_rotation);

    // Convert to Euler angles to separate axes
    tf2::Matrix3x3 mat(original_rotation);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw); 

    // Rebuild the quaternion using ONLY the pitch (Y-axis rotation)
    tf2::Quaternion y_only_rotation;
    y_only_rotation.setRPY(0.0, pitch, 0.0);

    // 3. Setup Transform
    tf2::Transform transform_W_D;
    transform_W_D.setOrigin(translation);
    transform_W_D.setRotation(y_only_rotation);

    // 4. Create Waypoint Vector
    tf2::Vector3 waypoint(waypoint_DCS.x, waypoint_DCS.y, waypoint_DCS.z);

    // 5. Apply Full Transformation (Rotation + Translation)
    // FIX: Using transform operator instead of 'translation + waypoint'
    tf2::Vector3 waypoint_WCS = transform_W_D * waypoint;
    
    // 6. Construct output
    geometry_msgs::msg::Point p;
    p.x = waypoint_WCS.x();
    p.y = waypoint_WCS.y();
    p.z = waypoint_WCS.z();
    
    return p;
}


void Planner::update_waypoint(void) {
    printf("Updating Waypoint\n Number of midpoints: %zu\n", waypoints_W.size());
    for (const auto& waypoint : waypoints_W) {
        printf("Midpoint: %f %f %f\n", waypoint.x, waypoint.y, waypoint.z);
    }

    
    if ((!waypoints_W.empty()) && _position.has_value()) { //first time this fails
        //convert to WCS and add to tree
        for (const auto& waypoint_W : waypoints_W){
            printf("Waypoint W: %f %f %f\n", waypoint_W.x, waypoint_W.y, waypoint_W.z);
            if( waypoint_W.x < _position.value().x ) {
                //add elements to waypoints tree "waypoints"
                // waypoints.add(waypoint_W)

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

    // 1. Check waypoint distance (X-Z plane only)
    double dx = current_waypoint.point.x - _position.value().x;
    double dz = current_waypoint.point.z - _position.value().z;
    double dist = std::sqrt(dx*dx + dz*dz);
    
    if(dist < 0.5){ // Threshold
        update_waypoint();
        // Recalculate dx, dz, and dist for the new waypoint
        dx = current_waypoint.point.x - _position.value().x;
        dz = current_waypoint.point.z - _position.value().z;
        dist = std::sqrt(dx*dx + dz*dz);
    }

    cmd.y = 0.00;

    printf("Current Waypoint: %f %f %f\n", current_waypoint.point.x, current_waypoint.point.y, current_waypoint.point.z);
    printf("Current Position: %f %f %f\n", _position.value().x, _position.value().y, _position.value().z);

    // Check if current_waypoint is valid
    if (std::abs(current_waypoint.point.x) > 0.001 && std::abs(current_waypoint.point.z) > 0.001) { 
        
        // --- DISTANCE ERROR (Magnitude) ---
        // 'dist' is exactly the magnitude of the distance in the X-Z plane
        double distance_error = dist;

        // --- HEADING ERROR (Rotation) ---
        // 1. Calculate intended global heading towards waypoint in X-Z plane
        double target_heading = std::atan2(dx, dz);

        // 2. Get current global heading (Y-axis rotation / Pitch)
        tf2::Quaternion rot_W;
        tf2::fromMsg(_orientation_abs, rot_W);
        tf2::Matrix3x3 mat(rot_W);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw); 
        double current_heading = pitch; 

        // 3. Calculate heading error
        double heading_error = target_heading - current_heading;

        // 4. Normalize the angle to be strictly between -PI and PI
        // This prevents the drone from spinning the long way around
        while (heading_error > M_PI)  heading_error -= 2.0 * M_PI;
        while (heading_error < -M_PI) heading_error += 2.0 * M_PI;


        cmd.x = .005 * distance_error; // Forward tilt based on distance
        cmd.w = .5 * heading_error;   // Spin based on heading difference
        cmd.z = 0.1; // Kept from your original code
        
        // --- LIMITERS ---
        if(cmd.x > 0.1) cmd.x = 0.1;
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
