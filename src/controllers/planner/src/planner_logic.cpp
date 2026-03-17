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
        if (dist > 1.1) {
            cmd.x = 0.04 * local_x;         // Move forward based on local X error
            cmd.w = 0.95 * target_yaw_diff; // Turn based on angular error

            // Optional: Limit speeds so it doesn't flip
            cmd.x = std::clamp(cmd.x, -0.15, 0.15);
            cmd.w = std::clamp(cmd.w, -0.6, 0.6);
            _lock_orientation = _imu.orientation;
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
    cmd.x = std::clamp(cmd.x, -0.2, 0.2);
    cmd.z = std::clamp(cmd.z, -0.2, 0.2);
    cmd.w = std::clamp(cmd.w, -0.6, 0.6);

    this->get_clock()->now();
    _movement_pub->publish(cmd);

    RCLCPP_INFO(
        this->get_logger(),
        "SCAN LOCK: YawErr: %.3f | LocalErr: X:%.2f Y:%.2f | LocalV: X:%.2f | %.4f",
        yaw_error, local_x, local_y, local_vx, (this->now() - _nav_time).seconds());

    if ((this->now() - _nav_time).seconds() > 4.5) {
        RCLCPP_INFO(this->get_logger(), "Triggered Scan Analysis");

        _nav_mode = eNavMode::eScanning;
        _nav_time = this->now();
    }
}

void Planner::_nav_mode_scan(void) {
    RCLCPP_INFO(this->get_logger(), "SCANNING: Projecting RRT Step...");

    if (_open_markers.points.empty()) {
        _nav_mode = eNavMode::eBacktracking;
        _nav_time = this->now();
        return;
    }

    auto rpy = quat_to_rot(_imu.orientation);
    double cos_y = std::cos(rpy[2]);
    double sin_y = std::sin(rpy[2]);

    NavTree::nav_node_t* best_new_node = nullptr;
    double min_angle_to_center = 180.0; // To pick the gap most "in front" of us
    const double RRT_STEP_SIZE = 1.2;

    for (size_t i = 0; i + 1 < _open_markers.points.size(); i += 2) {
        const auto& p1 = _open_markers.points[i];
        const auto& p2 = _open_markers.points[i + 1];

        // 1. Find the local midpoint of the gap
        double mid_x = (p1.x + p2.x) / 2.0;
        double mid_y = (p1.y + p2.y) / 2.0;

        // 2. Filter: Ignore if behind us
        if (mid_x < 0.0)
            continue;

        // 3. RRT PROJECTED STEP
        // Calculate the unit vector toward the gap midpoint
        double dist_to_gap = std::hypot(mid_x, mid_y);
        if (dist_to_gap < 0.5)
            continue; // Skip gaps we are already inside

        // Project a point exactly 2m away in that direction
        double step_local_x = (mid_x / dist_to_gap) * RRT_STEP_SIZE;
        double step_local_y = (mid_y / dist_to_gap) * RRT_STEP_SIZE;

        // 4. Global Transformation
        geometry_msgs::msg::Point global_pt;
        global_pt.x = (step_local_x * cos_y - step_local_y * sin_y) + _position.x;
        global_pt.y = (step_local_x * sin_y + step_local_y * cos_y) + _position.y;
        global_pt.z = 0.0;

        // 5. Add to Tree
        // We pick the "best" node based on which one is most aligned with our nose
        double angle_err = std::abs(std::atan2(step_local_y, step_local_x));

        NavTree::nav_node_t* pNode = _navtree.add_node(global_pt, _nav_target);
        if (pNode && angle_err < min_angle_to_center) {
            min_angle_to_center = angle_err;
            best_new_node = pNode;
        }
    }

    // 6. Transition
    if (best_new_node) {
        _nav_target = best_new_node;
        _nav_mode = eNavMode::eNavigating;
        RCLCPP_INFO(this->get_logger(), "Stepping 2m toward frontier.");
    } else {
        _nav_mode = eNavMode::eBacktracking;
        RCLCPP_INFO(this->get_logger(), "Path blocked or duplicate. Backtracking.");
    }

    _nav_time = this->now();
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
