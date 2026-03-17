/**
 * @file planner_logic.cpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2026-03-16
 * @modified Last Modified: 2026-03-16
 *
 * @copyright Copyright (c) 2026
 */

#include "planner/planner.hpp"
#include "planner/quatrot.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>

void Planner::_nav_mode_init(void) {
    // Check if the tree is empty
    // if (!_navtree.get_n()) {
    // Add node to start moving forwards into tunnel
    geometry_msgs::msg::Point p;
    p.x = 2 + _position.x;
    p.y = 0 + _position.y;
    p.z = 0;
    _navtree.add_node(p, NULL);
    // }
    // Set the target to the closest node
    _nav_target = _navtree.get_nearest(_position);
    _nav_mode = eNavMode::eNavigating;
}

void Planner::_nav_mode_nav(void) {
    geometry_msgs::msg::Quaternion cmd;
    cmd.x = 0.0;
    cmd.y = 0.0;
    cmd.z = 0.0;
    cmd.w = 0.0;

    if (_nav_target) {
        // 1. Get Global Delta
        double dx = _nav_target->x - _position.x;
        double dy = _nav_target->y - _position.y;
        double dist = std::hypot(dx, dy);

        // 2. Get Current Yaw (using your quat_to_rot fixed for Unity/ROS)
        auto rpy = quat_to_rot(_imu.orientation);
        double yaw = rpy[2] + 0.05;

        // 3. Transform Global Delta to Local Drone Frame
        // This is the key: it tells us how much to move 'forward' and 'sideways'
        double local_x = dx * std::cos(yaw) + dy * std::sin(yaw);
        double local_y = -dx * std::sin(yaw) + dy * std::cos(yaw);

        // 4. Calculate Heading Error (How much do we need to turn?)
        double target_yaw_diff = -std::atan2(local_y, local_x);

        // 5. Control Gains
        // cmd.x = Pitch (Forward/Back). cmd.w = Yaw Rate.
        if (dist > 0.5) {
            cmd.x = 0.02 * local_x;         // Move forward based on local X error
            cmd.w = 0.95 * target_yaw_diff; // Turn based on angular error

            // Optional: Limit speeds so it doesn't flip
            cmd.x = std::clamp(cmd.x, -0.1, 0.1);
            cmd.w = std::clamp(cmd.w, -0.6, 0.6);
            _lock_orientation = _imu.orientation;
        } else {
            // Arrival Logic: Mark as visited and find next
            _navtree.set_visited(_nav_target, true);
            _nav_mode = eNavMode::eScanning;
            // _nav_target = nullptr; // Next loop will trigger search for new target
        }
    }

    RCLCPP_INFO(this->get_logger(), "Target Dist: %.2f | Ctrl: P:%.2f Y:%.2f",
                _nav_target ? std::hypot(_nav_target->x - _position.x,
                                         _nav_target->y - _position.y)
                            : 0.0,
                cmd.x, cmd.w);

    _movement_pub->publish(cmd);
}

void Planner::_nav_mode_scan(void) {
    if (!_nav_target)
        return;

    // 1. Get Current Yaw
    auto rpy_now = quat_to_rot(_imu.orientation);
    double current_yaw = rpy_now[2];

    // 2. Rotation Error (Locked Heading vs Current)
    auto rpy_lock = quat_to_rot(_lock_orientation);
    double yaw_error = rpy_lock[2] - current_yaw;

    // Normalize yaw error to [-PI, PI]
    while (yaw_error > M_PI)
        yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI)
        yaw_error += 2.0 * M_PI;

    // 3. Position Error (Global to Local)
    double dx = _nav_target->x - _position.x;
    double dy = _nav_target->y - _position.y;

    double cos_y = std::cos(current_yaw);
    double sin_y = std::sin(current_yaw);

    // Transform global displacement into the drone's current local frame
    double local_x = dx * cos_y + dy * sin_y;
    double local_y = -dx * sin_y + dy * cos_y;

    // 4. Controller Application
    geometry_msgs::msg::Quaternion cmd;
    cmd.y = 0.0; // Altitude hold

    // Apply your specific gains
    // cmd.w (Yaw) targets the lock orientation
    cmd.w = -0.95 * yaw_error;

    // cmd.x (Pitch) and cmd.z (Roll) hold the XY position
    cmd.x = 0.05 * local_x;
    cmd.z = 0.05 * local_y;

    // 5. Safety Clamps
    cmd.x = std::clamp(cmd.x, -0.2, 0.2);
    cmd.z = std::clamp(cmd.z, -0.2, 0.2);
    cmd.w = std::clamp(cmd.w, -0.6, 0.6);

    _movement_pub->publish(cmd);

    RCLCPP_INFO(this->get_logger(), "SCAN LOCK: YawErr: %.3f | LocalErr: X:%.2f Y:%.2f",
                yaw_error, local_x, local_y);
}

void Planner::_nav_mode_backtrack(void) {
}

void Planner::ctrl_callback(void) {

    switch (_nav_mode) {
    case eNavMode::eInit:
        _nav_mode_init();
        break;
    case eNavMode::eNavigating:
        _nav_mode_nav();
        break;
    case eNavMode::eScanning:
        _nav_mode_scan();
        break;
    case eNavMode::eBacktracking:
        _nav_mode_backtrack();
        break;
    default:
        _nav_mode = eNavMode::eInit;
        break;
    }

    auto rot = quat_to_rot(_imu.orientation);
    _navtree_pub->publish(
        _navtree.get_all(_position, rot[2], _open_markers.header.frame_id));
}
