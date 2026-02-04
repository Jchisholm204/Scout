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

#include "localization/localization.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>

Localization::Localization() : Node("ldr_local") {
    this->declare_parameter("vert_lidar_topic", "cb/ls_vertical");
    this->declare_parameter("front_lidar_topic", "cb/ls_front");
    this->declare_parameter("ldr_segment_topic", "/segments");

    std::string vert_lidar_topic = this->get_parameter("vert_lidar_topic").as_string();
    std::string front_lidar_topic = this->get_parameter("front_lidar_topic").as_string();
    std::string ldr_segment_topic = this->get_parameter("ldr_segment_topic").as_string();

    // Create Subscriptions to drone topics
    _vertical_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        vert_lidar_topic, 10,
        std::bind(&Localization::_vldr_callback, this, std::placeholders::_1));
    _horizontal_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        front_lidar_topic, 10,
        std::bind(&Localization::_hldr_callback, this, std::placeholders::_1));
    _hldr_seg_sub = this->create_subscription<slg_msgs::msg::SegmentArray>(
        ldr_segment_topic, 10,
        std::bind(&Localization::_hldr_seg_callback, this, std::placeholders::_1));

    // Create Publishers
    _wall_marker_pub =
        this->create_publisher<visualization_msgs::msg::Marker>("/wall_markers", 10);
    _open_marker_pub =
        this->create_publisher<visualization_msgs::msg::Marker>("/open_markers", 10);
}

Localization::~Localization() {
}

void Localization::_vldr_callback(const sensor_msgs::msg::LaserScan& ldr) {
    (void) ldr;
}

void Localization::_hldr_callback(const sensor_msgs::msg::LaserScan& ldr) {
    (void) ldr;
}

void Localization::_hldr_seg_callback(const slg_msgs::msg::SegmentArray& seg_msg) {
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
            } else {
                wall_list.points.push_back(pA_end);   // Last point of current line
                wall_list.points.push_back(pB_start); // First point of NEXT line
            }
        }
    }

    _wall_marker_pub->publish(wall_list);
    _open_marker_pub->publish(gap_list);
}
