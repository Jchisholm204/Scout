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

#include "gravitational/gravitational.hpp"

#include <tf2/LinearMath/Quaternion.h>

#define ZERO_VEC(vec)                                                                    \
    (vec).x = 0;                                                                         \
    (vec).y = 0;                                                                         \
    (vec).z = 0;

#include <unistd.h>

Gravitational::Gravitational() : Node("gravity_ctrl") {
    this->declare_parameter("imu_topic", "/inav/imu");
    this->declare_parameter("gyro_topic", "/inav/gyro");
    this->declare_parameter("battery_topic", "/inav/batt");
    this->declare_parameter("motor_topic", "/inav/mtr");
    this->declare_parameter("altitude_topic", "/inav/altitude");
    this->declare_parameter("hor_lidar_topic", "/lidar");
    this->declare_parameter("vrt_lidar_topic", "/veridar");
    this->declare_parameter("target_topic", "/gtarget");
    this->declare_parameter("movement_topic", "/inav/movement");
    this->declare_parameter("arming_topic", "/inav/armed");
    this->declare_parameter("collide_max", 0.8f);
    this->declare_parameter("hover_alt", 0.5f);
    this->declare_parameter("log_level", "none");
    this->declare_parameter("horizontal_weight", 1);
    this->declare_parameter("vertical_weight", 1);
    this->declare_parameter("target_weight", 1);
    this->declare_parameter("hover_weight", 1);

    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    std::string gyro_topic = this->get_parameter("gyro_topic").as_string();
    std::string battery_topic = this->get_parameter("battery_topic").as_string();
    std::string motor_topic = this->get_parameter("motor_topic").as_string();
    std::string altitude_topic = this->get_parameter("altitude_topic").as_string();
    std::string horscan_topic = this->get_parameter("hor_lidar_topic").as_string();
    std::string verscan_topic = this->get_parameter("vrt_lidar_topic").as_string();
    std::string target_topic = this->get_parameter("target_topic").as_string();
    std::string movement_topic = this->get_parameter("movement_topic").as_string();
    std::string arming_topic = this->get_parameter("arming_topic").as_string();
    _collide_max = this->get_parameter("collide_max").as_double();
    _hover_alt = this->get_parameter("hover_alt").as_double();
    std::string log_level = this->get_parameter("log_level").as_string();
    _w_hor = this->get_parameter("horizontal_weight").as_double();
    _w_ver = this->get_parameter("vertical_weight").as_double();
    _w_trg = this->get_parameter("target_weight").as_double();
    _w_hover = this->get_parameter("hover_weight").as_double();

    // Sanitize Parameters
    if (((int) _collide_max) == -1) {
        RCLCPP_WARN(this->get_logger(), "`collide_max` is ignored");
        _collide_max = FLT_MAX;
    } else if (_collide_max < 0) {
        std::cerr << "Invalid Parameter: `collide_max`\n Possible options are:\n"
                  << " - `-1` (considers everything in the enviroment)" << std::endl
                  << " - (`0`, `15`]" << std::endl;
        throw std::runtime_error("collide_max invalid argument");
    }

    // Parse Log Level
    if (log_level == "none") {
        _log_level = LogLevel::None;
    } else if (log_level == "info") {
        _log_level = LogLevel::Info;
    } else if (log_level == "warning") {
        _log_level = LogLevel::Warning;
    } else if (log_level == "error") {
        _log_level = LogLevel::Error;
    } else {
        std::cerr
            << "Log Level: [" << log_level << "] is unknown. " << std::endl
            << "Possible options are:\n - [none]\n - [info]\n - [warning]\n - [error]"
            << std::endl;
        throw std::runtime_error("log_level unknown");
    }

    // Create Subscriptions to drone topics
    _imu_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
        imu_topic, 10,
        std::bind(&Gravitational::_imu_callback, this, std::placeholders::_1));
    _gyro_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
        gyro_topic, 10,
        std::bind(&Gravitational::_gyro_callback, this, std::placeholders::_1));
    _battery_sub = this->create_subscription<sensor_msgs::msg::BatteryState>(
        battery_topic, 10,
        std::bind(&Gravitational::_batt_callback, this, std::placeholders::_1));
    _motor_sub = this->create_subscription<std_msgs::msg::Float32>(
        motor_topic, 10,
        std::bind(&Gravitational::_motor_callback, this, std::placeholders::_1));
    _altitude_sub = this->create_subscription<std_msgs::msg::Float32>(
        motor_topic, 10,
        std::bind(&Gravitational::_altitude_callback, this, std::placeholders::_1));
    _vertical_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        verscan_topic, 10,
        std::bind(&Gravitational::_vldr_callback, this, std::placeholders::_1));
    _horizontal_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        horscan_topic, 10,
        std::bind(&Gravitational::_hldr_callback, this, std::placeholders::_1));
    _target_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
        target_topic, 10,
        std::bind(&Gravitational::_target_callback, this, std::placeholders::_1));

    // Create Publishers
    _movement_pub =
        this->create_publisher<geometry_msgs::msg::Quaternion>(movement_topic, 10);
    _arming_pub = this->create_publisher<std_msgs::msg::Bool>(arming_topic, 10);
    _pose_pub =
        this->create_publisher<visualization_msgs::msg::Marker>(target_topic + ".mk", 10);

    // Zero out internal parameters
    _mtr_speed = 0;
    _drone_alt = 0;
    ZERO_VEC(_g_targ);
    ZERO_VEC(_g_gyro);
    ZERO_VEC(_g_horizontal);
    ZERO_VEC(_g_vertical);
    ZERO_VEC(_g_realsense);
    ZERO_VEC(_g_last);

    // Setup operational timer
    _ctrl_timer = this->create_wall_timer(std::chrono::milliseconds(50),
                                          std::bind(&Gravitational::ctrl_callback, this));
}

void Gravitational::_imu_callback(const geometry_msgs::msg::Vector3& imu) {
    // Future Use: Acceleration limits in the real world
    (void) (imu);
}
void Gravitational::_gyro_callback(const geometry_msgs::msg::Vector3& gyro) {
    // Future Use: Internal PID Controller?
    _g_gyro = gyro;
}
void Gravitational::_batt_callback(const sensor_msgs::msg::BatteryState& batt) {
    // Future Use: Compensate for battery drop
    (void) (batt);
}
void Gravitational::_motor_callback(const std_msgs::msg::Float32& mtr) {
    // Future Use: Current Power Calculations
    _mtr_speed = mtr.data;
}
void Gravitational::_altitude_callback(const std_msgs::msg::Float32& alt) {
    // Future Use: Use as source for Fz
    _drone_alt = alt.data;
}
void Gravitational::_vldr_callback(const sensor_msgs::msg::LaserScan& ldr) {
    // Zero out the vector
    ZERO_VEC(_g_vertical);

    // Loop through all LiDAR Points
    for (size_t i = 0; i < ldr.ranges.size(); i++) {
        // Calcuate angle and distance of measurement
        float angle = ldr.angle_min + i * ldr.angle_increment;
        float range = ldr.ranges[i];
        // Adjust the value to max range [0, _collide_max]
        range = range > _collide_max ? 0 : _collide_max - range;
        // Calcuate the confidence in the reading [0, 1]
        float confidence = ldr.intensities[i] / 255.0f;
        // Calcuate the resultant force
        float F = range * confidence;
        // Calcuate the perpendicular forces
        float Fx = F * cos(angle) / _collide_max;
        float Fy = F * sin(angle) / _collide_max;
        // Add or subtract depending on the side of the axis [-PI, PI]
        if (angle < 0) {
            _g_vertical.x += Fx;
            _g_vertical.y += Fy;
        } else {
            _g_vertical.x -= Fx;
            _g_vertical.y -= Fy;
        }
    }
}
void Gravitational::_hldr_callback(const sensor_msgs::msg::LaserScan& ldr) {
    // Zero out the vector
    ZERO_VEC(_g_horizontal);

    // Loop through all LiDAR Points
    for (size_t i = 0; i < ldr.ranges.size(); i++) {
        // Calcuate angle and distance of measurement
        float angle = ldr.angle_min + i * ldr.angle_increment;
        float range = ldr.ranges[i];
        // Adjust the value to max range [0, _collide_max]
        range = range > _collide_max ? 0 : _collide_max - range;
        // Calcuate the confidence in the reading [0, 1]
        float confidence = ldr.intensities[i] / 255.0f;
        // Calcuate the resultant force
        float F = range * confidence;
        // Calcuate the perpendicular forces
        float Fx = F * cos(angle) / _collide_max;
        float Fy = F * sin(angle) / _collide_max;
        // Add or subtract depending on the side of the axis [-PI, PI]
        if (angle < 0) {
            _g_horizontal.x += Fx;
            _g_horizontal.y += Fy;
        } else {
            _g_horizontal.x -= Fx;
            _g_horizontal.y -= Fy;
        }
    }
}
void Gravitational::_target_callback(const geometry_msgs::msg::Vector3& target) {
    // Callback to load the next target (point in 3D space)
    _g_targ = target;
}

void Gravitational::ctrl_callback(void) {
    // Always arm (for now)
    std_msgs::msg::Bool armed;
    armed.data = true;
    _arming_pub->publish(armed);

    // Accumulate (Sum) The Forces
    float Fx = 0, Fy = 0, Fz = 0;
    Fx += _g_horizontal.x * _w_hor + _g_vertical.x * _w_ver + _g_targ.x * _w_trg;
    Fy += _g_horizontal.y * _w_hor + _g_vertical.y * _w_ver + _g_targ.y * _w_trg;
    Fz += _g_horizontal.z * _w_hor + _g_vertical.z * _w_ver + _g_targ.z * _w_trg +
          _hover_alt * _w_hover;

    // Normalize the Forces Vector
    float F = sqrt(pow(Fx, 2) + pow(Fy, 2) + pow(Fz, 2));
    Fx /= F;
    Fy /= F;
    Fz /= F;

    // Save this reading
    geometry_msgs::msg::Vector3 c;
    c.x = Fx;
    c.y = Fy;
    c.z = Fz;

    // Average with the last value
    Fx = (Fx + _g_last.x) / 2;
    Fy = (Fy + _g_last.y) / 2;
    Fz = (Fz + _g_last.z) / 2;

    // Do not perpetuate the average
    _g_last = c;

    // Fill Movement Message
    geometry_msgs::msg::Quaternion q;
    q.x = Fx;
    q.y = Fy;
    q.z = Fz;
    q.w = 0;
    _movement_pub->publish(q);

    // Fill pose message
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = this->now();
    m.ns = "vector";
    m.type = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;

    // Origin of the drone is (0, 0, 0)
    geometry_msgs::msg::Point start;
    start.x = 0;
    start.y = 0;
    start.x = 0;
    // Place the Vector
    geometry_msgs::msg::Point end;
    end.x = Fx;
    end.y = Fy;
    end.z = Fz;
    // Add the points to the pose message
    m.points.push_back(start);
    m.points.push_back(end);

    if (_log_level >= LogLevel::Info)
        RCLCPP_INFO(get_logger(), "x: %0.2f y: %0.2f z: %0.2f = %0.2f\n", Fx, Fy, Fz, F);

    // Appearance
    m.scale.x = 0.02; // shaft diameter
    m.scale.y = 0.05; // head diameter
    m.scale.z = 0.05; // head length

    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;

    _pose_pub->publish(m);

    // RCLCPP_INFO(this->get_logger(),
    //             "Roll: %.2f° Pitch: %.2f° -> q=(%.2f, %.2f, %.2f, %.2f)",
    //             roll * 180.0 / M_PI, pitch * 180.0 / M_PI, q.x(), q.y(), q.z(), q.w());
}
