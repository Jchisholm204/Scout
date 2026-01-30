/**
 * @file rplidar_bridge.cpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2025-09-18
 * @modified Last Modified: 2025-09-18
 *
 * @copyright Copyright (c) 2025
 */

#include "control/control_node.hpp"

#include <arpa/inet.h>
#include <cfloat>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

ControlNode::ControlNode() : Node("sim_control") {
    RCLCPP_INFO(this->get_logger(), "Liftoff Control Node Online");
    this->declare_parameter("ctrl_topic", "cb/out/vel");
    std::string ctrl_topic = this->get_parameter("ctrl_topic").as_string();
    joystick_init(&_joy);
    _vel_cmd_sub = this->create_subscription<geometry_msgs::msg::Quaternion>(
        ctrl_topic, 10,
        std::bind(&ControlNode::_vel_cmd_callback, this, std::placeholders::_1));
}

ControlNode::~ControlNode() {
    joystick_close(&_joy);
}

void ControlNode::_vel_cmd_callback(const geometry_msgs::msg::Quaternion& qt) {
    joystick_write(&_joy, AXIS_THROTTLE, qt.z * AXIS_MAX);
    joystick_write(&_joy, AXIS_YAW, qt.w * AXIS_MAX);
    joystick_write(&_joy, AXIS_PITCH, qt.y * AXIS_MAX);
    joystick_write(&_joy, AXIS_ROLL, qt.x * AXIS_MAX);
}
