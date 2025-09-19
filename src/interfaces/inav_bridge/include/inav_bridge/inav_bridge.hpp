/**
 * @file inav_bridge.hpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief 
 * @version 0.1
 * @date Created: 2025-09-17
 * @modified Last Modified: 2025-09-17
 *
 * @copyright Copyright (c) 2025
 */


#ifndef _INAV_BRIDGE_HPP_
#define _INAV_BRIDGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "FlightController.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

class INAVBridge : public rclcpp::Node {
    public:
        INAVBridge();
    private:
        fcu::FlightController _fc;
        rclcpp::TimerBase::SharedPtr _timer;
        void timer_callback(void);
        void imu_callback(const msp::msg::RawImu& imu);
        std::shared_ptr<tf2_ros::TransformBroadcaster> _tfbcast;
        rclcpp::TimerBase::SharedPtr _timer_tfbcast;
        void tfbcast_callback(void);
        std::shared_ptr<msp::msg::ImuSI> _imu;
};

#endif

