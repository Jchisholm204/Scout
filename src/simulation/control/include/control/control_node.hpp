/**
 * @file control_node.hpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2026-01-13
 * @modified Last Modified: 2026-01-13
 *
 * @copyright Copyright (c) 2026
 */

#ifndef _CONTROL_NODE_HPP_
#define _CONTROL_NODE_HPP_

#include "joystick.h"

#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();
    ~ControlNode();

  private:
    struct joystick _joy;
    rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr _vel_cmd_sub;
    void _vel_cmd_callback(const geometry_msgs::msg::Quaternion& qt);
};

#endif
