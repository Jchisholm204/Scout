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

#include <cmath>
#include <math.h>
#include <slg_msgs/msg/segment_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>

Localization::Localization() : Node("ldr_local") {
    this->declare_parameter("vert_lidar_topic", "sim/ls_vertical");
    this->declare_parameter("front_lidar_topic", "sim/ls_front");
    this->declare_parameter("ldr_segment_topic", "/segments");
    this->declare_parameter("line_gap_thresh", _line_gap_thresh);
    this->declare_parameter("line_angle_thresh", _line_angle_thresh);

    std::string vert_lidar_topic = this->get_parameter("vert_lidar_topic").as_string();
    std::string front_lidar_topic = this->get_parameter("front_lidar_topic").as_string();
    std::string ldr_segment_topic = this->get_parameter("ldr_segment_topic").as_string();
    _line_gap_thresh = this->get_parameter("line_gap_thresh").as_double();
    _line_angle_thresh = this->get_parameter("line_angle_thresh").as_double();

    // Create Subscriptions to drone topics
    _vertical_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        vert_lidar_topic, 10,
        std::bind(&Localization::_vldr_callback, this, std::placeholders::_1));
    _horizontal_scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        front_lidar_topic, 10,
        std::bind(&Localization::_hldr_callback, this, std::placeholders::_1));

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
    visualization_msgs::msg::Marker walls = _process_ldr(ldr);
    this->_wall_marker_pub->publish(walls);
    visualization_msgs::msg::Marker open;
    open.header = walls.header;
    open.ns = "open_space";
    open.id = 0;
    open.type = visualization_msgs::msg::Marker::LINE_LIST;
    open.scale.x = 0.05;
    open.color.g = 1.0;
    open.color.a = 1.0;
    geometry_msgs::msg::Point last = walls.points[0];
    for (size_t i = 0; i + 3 < walls.points.size(); i += 2) {
    const auto& p_end_current = walls.points[i+1];
    const auto& p_start_next = walls.points[i+2];

    // Calculate distance between segments
    float dx = p_start_next.x - p_end_current.x;
    float dy = p_start_next.y - p_end_current.y;
    float gap_dist = sqrt(dx*dx + dy*dy);

    if (gap_dist > _line_gap_thresh) {
        open.points.push_back(p_end_current);
        open.points.push_back(p_start_next);
    }
}
    this->_open_marker_pub->publish(open);
}

visualization_msgs::msg::Marker Localization::_process_ldr(
    const sensor_msgs::msg::LaserScan& ldr) {
    visualization_msgs::msg::Marker walls;
    walls.header = ldr.header;
    walls.ns = "detected_walls";
    walls.id = 0;
    walls.type = visualization_msgs::msg::Marker::LINE_LIST;
    walls.scale.x = 0.05;
    walls.color.r = 1.0;
    walls.color.a = 1.0;

    geometry_msgs::msg::Point start, first_pt;
    bool segment_started = false;
    bool first_point_captured = false;
    float range_last = 0.0, first_range = 0.0;
    int last_valid_idx = 0;

    for (size_t i = 0; i < ldr.ranges.size(); i++) {
        float range = ldr.ranges[i];

        // 1. VALIDATION CHECK
        if (!std::isfinite(range) || range < ldr.range_min || range > ldr.range_max) {
            // We do NOT reset segment_started yet. 
            // We just skip this index and try to connect the next valid one.
            continue;
        }

        // Calculate current point coordinates
        geometry_msgs::msg::Point current_pt;
        float angle = ldr.angle_min + (i * ldr.angle_increment);
        current_pt.x = range * cos(angle);
        current_pt.y = range * sin(angle);
        current_pt.z = 0;

        if (!first_point_captured) {
            first_pt = current_pt;
            first_range = range;
            first_point_captured = true;
        }

        // 2. SEGMENT INITIALIZATION
        if (!segment_started) {
            start = current_pt;
            range_last = range;
            last_valid_idx = i;
            segment_started = true;
            continue;
        }

        // 3. GEOMETRY MATH
        // Since we might have skipped points, we need the actual angle difference
        int steps = i - last_valid_idx;
        float angle_diff = steps * ldr.angle_increment;
        float cos_diff = cos(angle_diff);
        float sin_diff = sin(angle_diff);

        float range_last_2 = range_last * range_last;
        float range_2 = range * range;

        // Law of Cosines using the actual angle difference
        float line_gap = sqrt(std::max(0.0f, (range_2 + range_last_2) - 
                                             (2 * range * range_last * cos_diff)));

        // 4. BREAK CONDITIONS
        if (line_gap > _line_gap_thresh) {
            // Even if we skip invalid points, the resulting gap is too wide.
            // Reset the segment to start here.
            start = current_pt;
        } else {
            // For the angle threshold, we use the specific sin of this jump
            float line_angle = asin((range * sin_diff) / std::max(line_gap, 0.001f));

            if (line_angle > _line_angle_thresh) {
                walls.points.push_back(start);
                walls.points.push_back(current_pt);
                start = current_pt;
            }
        }

        range_last = range;
        last_valid_idx = i;
    }

    // --- WRAP AROUND LOGIC ---
    // If the last point of the scan and the first point of the scan are both valid
    if (segment_started && first_point_captured) {

        // Calculate the gap between the last point (range_last) and the first
        // (first_range) The angle between them is usually the ldr.angle_increment (or 2pi
        // - total_angle)
        float cos_inc = cos(ldr.angle_increment);
        float range_2 = first_range * first_range;
        float range_last_2 = range_last * range_last;

        float wrap_gap =
            sqrt(std::max(0.0f, (range_2 + range_last_2) -
                                    (2 * first_range * range_last * cos_inc)));

        // Only connect if they are close enough to be the same wall
        if (wrap_gap < _line_gap_thresh) {
            walls.points.push_back(start);    // From the last "start" marker
            walls.points.push_back(first_pt); // Back to the very beginning
        }
    }

    return walls;
}

