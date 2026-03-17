/**
 * @file quatrot.hpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2026-03-16
 * @modified Last Modified: 2026-03-16
 *
 * @copyright Copyright (c) 2026
 */

#ifndef _QUATROT_HPP_
#define _QUATROT_HPP_

#include <geometry_msgs/msg/quaternion.hpp>
#include <vector>
#include <math.h>

/**
 * Converts a ROS Geometry Quaternion to Euler angles (Roll, Pitch, Yaw).
 * Returns values in Radians.
 */
static inline std::vector<double> quat_to_rot(const geometry_msgs::msg::Quaternion& q) {
    std::vector<double> rpy(3);

    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    rpy[0] = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        rpy[1] = std::copysign(M_PI / 2, sinp); // Use 90 degrees if out of range
    else
        rpy[1] = std::asin(sinp);

    // Yaw (z-axis rotation)
    // Original Yaw calculation
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double raw_yaw = std::atan2(siny_cosp, cosy_cosp);

    // HIGH LEVEL FIX:
    // If Forward is Left, we are offset by 90 degrees.
    // Adjust by PI/2 (90 deg) or -PI/2 depending on the specific Unity scene export.
    rpy[2] = raw_yaw - (M_PI / 2.0);

    // Normalize to keep within [-PI, PI]
    if (rpy[2] < -M_PI)
        rpy[2] += 2.0 * M_PI;
    if (rpy[2] > M_PI)
        rpy[2] -= 2.0 * M_PI;

    return rpy;
}

#endif
