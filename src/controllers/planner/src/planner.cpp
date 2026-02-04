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
#include <unistd.h>

Planner::Planner() : Node("path_planner") {
    this->declare_parameter("imu_topic", "sim/imu");
    this->declare_parameter("battery_topic", "sim/batt");
    this->declare_parameter("mode_topic", "cb/mode");
    this->declare_parameter("vert_lidar_topic", "cb/ls_vertical");
    this->declare_parameter("front_lidar_topic", "cb/ls_front");
    this->declare_parameter("ldr_segment_topic", "/segments");
    this->declare_parameter("position_topic", "sim/position");
    this->declare_parameter("velocity_topic", "sim/velocity");
    this->declare_parameter("movement_topic", "cb/vel_cmd");

    std::string imu_topic = this->get_parameter("imu_topic").as_string();
    std::string battery_topic = this->get_parameter("battery_topic").as_string();
    std::string mode_topic = this->get_parameter("mode_topic").as_string();
    std::string vert_lidar_topic = this->get_parameter("vert_lidar_topic").as_string();
    std::string front_lidar_topic = this->get_parameter("front_lidar_topic").as_string();
    std::string ldr_segment_topic = this->get_parameter("ldr_segment_topic").as_string();
    std::string position_topic = this->get_parameter("position_topic").as_string();
    std::string velocity_topic = this->get_parameter("velocity_topic").as_string();
    std::string movement_topic = this->get_parameter("movement_topic").as_string();

    // Create Subscriptions to drone topics
    _imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, 10, std::bind(&Planner::_imu_callback, this, std::placeholders::_1));
    _battery_sub = this->create_subscription<sensor_msgs::msg::BatteryState>(
        battery_topic, 10,
        std::bind(&Planner::_batt_callback, this, std::placeholders::_1));
    _mode_sub = this->create_subscription<std_msgs::msg::UInt8>(
        mode_topic, 10, std::bind(&Planner::_mode_callback, this, std::placeholders::_1));
    _vertical_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        vert_lidar_topic, 10,
        std::bind(&Planner::_vldr_callback, this, std::placeholders::_1));
    _horizontal_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        front_lidar_topic, 10,
        std::bind(&Planner::_hldr_callback, this, std::placeholders::_1));
    _hldr_seg_sub = this->create_subscription<slg_msgs::msg::SegmentArray>(
        ldr_segment_topic, 10,
        std::bind(&Planner::_hldr_seg_callback, this, std::placeholders::_1));
    _pos_sub = this->create_subscription<geometry_msgs::msg::Point>(
        position_topic, 10,
        std::bind(&Planner::_pos_callback, this, std::placeholders::_1));
    _vel_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
        velocity_topic, 10,
        std::bind(&Planner::_vel_callback, this, std::placeholders::_1));

    // Create Publishers
    _movement_pub =
        this->create_publisher<geometry_msgs::msg::Quaternion>(movement_topic, 10);
    _wall_marker_pub =
        this->create_publisher<visualization_msgs::msg::Marker>("/wall_markers", 10);
    _open_marker_pub =
        this->create_publisher<visualization_msgs::msg::Marker>("/open_markers", 10);

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

void Planner::_vldr_callback(const sensor_msgs::msg::LaserScan& ldr) {
    this->_vertical_lidar = ldr;
}

void Planner::_hldr_callback(const sensor_msgs::msg::LaserScan& ldr) {
    this->_front_lidar = ldr;
    visualization_msgs::msg::Marker gap_markers;
    gap_markers.header = ldr.header;
    gap_markers.ns = "open_areas";
    gap_markers.id = 100;
    gap_markers.type = visualization_msgs::msg::Marker::LINE_LIST;
    gap_markers.action = visualization_msgs::msg::Marker::ADD;

    // Green and slightly transparent to look like a "field"
    gap_markers.scale.x = 0.03;
    gap_markers.color.g = 1.0;
    gap_markers.color.a = 0.6;

    for (size_t i = 0; i < ldr.ranges.size() - 1; ++i) {
        double r1 = ldr.ranges[i];
        double r2 = ldr.ranges[i + 1];

        // Check if the points are valid (not inf/nan)
        bool p1_valid = std::isfinite(r1) && r1 > ldr.range_min;
        bool p2_valid = std::isfinite(r2) && r2 > ldr.range_min;

        // We draw a line if we find a transition from valid to invalid,
        // or a massive jump in distance (a "threshold" for a gap)
        double jump_threshold = 0.5; // 50cm jump indicates an opening

        if (p1_valid && p2_valid && std::abs(r1 - r2) > jump_threshold) {
            // Calculate Cartesian coordinates for both points
            double angle1 = ldr.angle_min + i * ldr.angle_increment;
            double angle2 = ldr.angle_min + (i + 1) * ldr.angle_increment;

            geometry_msgs::msg::Point p1, p2;
            p1.x = r1 * std::cos(angle1);
            p1.y = r1 * std::sin(angle1);
            p2.x = r2 * std::cos(angle2);
            p2.y = r2 * std::sin(angle2);

            gap_markers.points.push_back(p1);
            gap_markers.points.push_back(p2);
        }
    }
    // _open_marker_pub->publish(gap_markers);
}

void Planner::_hldr_seg_callback(const slg_msgs::msg::SegmentArray& seg_msg) {
    const auto& segments = seg_msg.segments;
    if (segments.empty())
        return;

    // 1. Setup Red Markers (Walls)
    visualization_msgs::msg::Marker wall_list;
    wall_list.header = seg_msg.header;
    wall_list.ns = "detected_walls";
    wall_list.id = 0;
    wall_list.type = visualization_msgs::msg::Marker::LINE_LIST;
    wall_list.scale.x = 0.05;
    wall_list.color.r = 1.0;
    wall_list.color.a = 1.0;

    // 2. Setup Green Markers (Gaps/Openings)
    visualization_msgs::msg::Marker gap_list;
    gap_list.header = seg_msg.header;
    gap_list.ns = "gaps";
    gap_list.id = 1;
    gap_list.type = visualization_msgs::msg::Marker::LINE_LIST;
    gap_list.scale.x = 0.03;
    gap_list.color.g = 1.0;
    gap_list.color.a = 0.8;

    size_t num_segments = segments.size();

    for (size_t i = 0; i < num_segments; ++i) {
        if (segments[i].points.size() < 2)
            continue;

        // --- DRAW THE CURRENT WALL (Line A) ---
        geometry_msgs::msg::Point pA_start, pA_end;
        pA_start.x = segments[i].points.front().x;
        pA_start.y = segments[i].points.front().y;
        pA_end.x = segments[i].points.back().x;
        pA_end.y = segments[i].points.back().y;

        wall_list.points.push_back(pA_start);
        wall_list.points.push_back(pA_end);

        // --- BRIDGE TO THE NEXT WALL (The "Loop Around" Logic) ---
        // (i + 1) % num_segments creates the wrap-around from last back to first
        size_t next_idx = (i + 1) % num_segments;

        if (segments[next_idx].points.size() >= 2) {
            geometry_msgs::msg::Point pB_start;
            pB_start.x = segments[next_idx].points.front().x;
            pB_start.y = segments[next_idx].points.front().y;

            // Calculate the distance of the "Gap"
            double dist = std::sqrt(std::pow(pB_start.x - pA_end.x, 2) +
                                    std::pow(pB_start.y - pA_end.y, 2));

            // Only bridge if it's a meaningful gap (e.g., > 0.4m for drone passage)
            // If it's too small, it's just a corner; if it's huge, it's a hallway.
            if (dist > 1.2) {
                gap_list.points.push_back(pA_end);   // Last point of current line
                gap_list.points.push_back(pB_start); // First point of NEXT line
            }
            else{
                wall_list.points.push_back(pA_end);   // Last point of current line
                wall_list.points.push_back(pB_start); // First point of NEXT line
            }
        }
    }

    _wall_marker_pub->publish(wall_list);
    _open_marker_pub->publish(gap_list);
}

void Planner::_pos_callback(const geometry_msgs::msg::Point& position) {
    this->_position = position;
}

void Planner::_vel_callback(const geometry_msgs::msg::Vector3& velocity) {
    this->_velocity = velocity;
}

void Planner::ctrl_callback(void) {
    static int f_cmd = 0;
    geometry_msgs::msg::Quaternion cmd;
    if (f_cmd == 0) {
        f_cmd = 1;
        cmd.x = 0.0;
    } else if (f_cmd == 1) {
        f_cmd = -1;
        cmd.x = 0.06;
    } else if (f_cmd == -1) {
        f_cmd = 0;
        cmd.x = -0.05;
    }
    cmd.y = 0.00;
    cmd.z = 0.00;
    cmd.w = 0.00;
    RCLCPP_INFO(this->get_logger(), "Ctrl: %.3f %.3f %.3f %.3f", cmd.x, cmd.y, cmd.z,
                cmd.w);
    _movement_pub->publish(cmd);
}
