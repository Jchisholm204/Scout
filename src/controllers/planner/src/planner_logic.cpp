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
    _nav_time = this->now();
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
        if (dist > 1.3) {
            cmd.x = 0.04 * local_x;         // Move forward based on local X error
            cmd.w = 0.95 * target_yaw_diff; // Turn based on angular error

            // Optional: Limit speeds so it doesn't flip
            cmd.x = std::clamp(cmd.x, -0.15, 0.15);
            cmd.w = std::clamp(cmd.w, -0.6, 0.6);
            NavTree::nav_node_t* pLast = _navtree.get_parent(_nav_target);
            if (pLast) {
                // Calculate the angle from the parent to the new target
                double dx = _nav_target->x - pLast->x;
                double dy = _nav_target->y - pLast->y;
                double path_angle = std::atan2(dy, dx);

                double side_parallel_yaw = path_angle + (M_PI / 2.0);

                // 3. Normalize to [-PI, PI]
                while (side_parallel_yaw > M_PI)
                    side_parallel_yaw -= 2.0 * M_PI;
                while (side_parallel_yaw < -M_PI)
                    side_parallel_yaw += 2.0 * M_PI;

                // Average in the current heading
                side_parallel_yaw =
                    side_parallel_yaw * 0.90 + quat_to_rot(_imu.orientation)[2] * 0.1;

                // 4. Update the Lock Orientation
                tf2::Quaternion q;
                q.setRPY(0, 0, side_parallel_yaw);
                _lock_orientation.x = q.x();
                _lock_orientation.y = q.y();
                _lock_orientation.z = q.z();
                _lock_orientation.w = q.w();
            } else {
                _lock_orientation = _imu.orientation;
            }
        } else {
            // Arrival Logic: Mark as visited and find next
            _navtree.set_visited(_nav_target, true);
            _nav_time = this->now();
            _nav_mode = eNavMode::eWaitForScan;
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

void Planner::_nav_mode_wait_for_scan(void) {
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

    double local_x = dx * cos_y + dy * sin_y;
    double local_y = -dx * sin_y + dy * cos_y;

    // 4. Velocity "Friction" (The D-term)
    // Transform global velocity into the drone's local frame
    double local_vx = _velocity.x * cos_y + _velocity.y * sin_y;
    double local_vy = -_velocity.x * sin_y + _velocity.y * cos_y;

    // 5. Controller Application
    geometry_msgs::msg::Quaternion cmd;
    cmd.y = 0.0;

    // Gains
    const double Kp = 0.05; // Your current P gain
    const double Kd = 0.03; // D gain (start around 0.03 - 0.05)

    // cmd.w (Yaw) targets the lock orientation
    cmd.w = -0.95 * yaw_error;

    // XY Hold with D-term: (Error * P) - (Velocity * D)
    cmd.x = (local_x * Kp) - (local_vx * Kd);
    cmd.z = (local_y * Kp) - (local_vy * Kd);

    // 6. Safety Clamps
    cmd.x = std::clamp(cmd.x, -0.2, 0.2) + 0.01;
    cmd.z = std::clamp(cmd.z, -0.2, 0.2);
    cmd.w = std::clamp(cmd.w, -0.6, 0.6);

    this->get_clock()->now();
    _movement_pub->publish(cmd);

    RCLCPP_INFO(
        this->get_logger(),
        "SCAN LOCK: YawErr: %.3f | LocalErr: X:%.2f Y:%.2f | LocalV: X:%.2f | %.4f",
        yaw_error, local_x, local_y, local_vx, (this->now() - _nav_time).seconds());

    if ((this->now() - _nav_time).seconds() > 3.95) {
        RCLCPP_INFO(this->get_logger(), "Triggered Scan Analysis");

        _nav_mode = eNavMode::eScanning;
        _nav_time = this->now();
    }
}

void Planner::_nav_mode_scan(void) {
    if (_open_markers.points.empty()) {
        _nav_mode = eNavMode::eBacktracking;
        return;
    }

    auto rpy = quat_to_rot(_imu.orientation);
    double current_yaw = rpy[2];
    double cos_y = std::cos(current_yaw);
    double sin_y = std::sin(current_yaw);

    NavTree::nav_node_t* best_node = nullptr;
    double highest_score = -1.0;

    const double RRT_STEP_SIZE = 1.25;
    const double ROBOT_RADIUS = 2.5;

    for (size_t i = 0; i + 1 < _open_markers.points.size(); i += 2) {
        const auto& p1 = _open_markers.points[i];
        const auto& p2 = _open_markers.points[i + 1];

        double gap_width = std::hypot(p1.x - p2.x, p1.y - p2.y);
        if (gap_width < (ROBOT_RADIUS * 2.0))
            continue;

        double mid_x = (p1.x + p2.x) / 2.0;
        double mid_y = (p1.y + p2.y) / 2.0;
        double dist_to_gap = std::hypot(mid_x, mid_y);

        // Bias: We want deep gaps that are somewhat in front of us
        double angle_to_gap = std::atan2(mid_y, mid_x);
        double score = dist_to_gap * std::cos(angle_to_gap * 0.5);

        if (score > highest_score) {
            // 1. POSITION CALCULATION
            double step_local_x = (mid_x / dist_to_gap) * RRT_STEP_SIZE;
            double step_local_y = (mid_y / dist_to_gap) * RRT_STEP_SIZE;

            // Apply the "Swing Wide" to the position
            if (std::abs(mid_y) > 0.3) {
                step_local_x += 0.2;
            }

            step_local_x += 0.9;

            // 2. TARGET HEADING (The Fix)
            // Instead of facing the target point, face the ACTUAL gap center.
            // This ensures that as we move, our sensors are rotating toward the opening.
            double target_yaw_local = std::atan2(mid_y, mid_x);
            double global_yaw = current_yaw + target_yaw_local;

            geometry_msgs::msg::Point global_pt;
            global_pt.x = (step_local_x * cos_y - step_local_y * sin_y) + _position.x;
            global_pt.y = (step_local_x * sin_y + step_local_y * cos_y) + _position.y;
            global_pt.z = 0.0;

            // Assuming your NavTree node can store a desired yaw
            NavTree::nav_node_t* pNode = _navtree.add_node(global_pt, _nav_target);
            if (pNode) {
                // pNode->yaw = global_yaw; // Store this for your controller!
                highest_score = score;
                best_node = pNode;
            }
        }
    }

    if (best_node) {
        _nav_target = best_node;
        _nav_mode = eNavMode::eNavigating;
    } else {
        _nav_mode = eNavMode::eBacktracking;
    }
}

void Planner::_nav_mode_backtrack(void) {
    _nav_mode = eNavMode::eNavigating;
}

void Planner::ctrl_callback(void) {

    switch (_nav_mode) {
    case eNavMode::eInit:
        _nav_mode_init();
        break;
    case eNavMode::eNavigating:
        _nav_mode_nav();
        break;
    case eNavMode::eWaitForScan:
        _nav_mode_wait_for_scan();
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
