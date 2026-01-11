/**
 * @file lidarstreams.hpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief 
 * @version 0.1
 * @date Created: 2026-01-10
 *
 * @copyright Copyright (c) 2026
 */

#ifndef _LIDARSTREAMS_HPP_
#define _LIDARSTREAMS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarStreams : public rclcpp::Node {
    public:
        LidarStreams();
        ~LidarStreams();
    private:
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _ls_front_pub;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _ls_vertical_pub;
        rclcpp::TimerBase::SharedPtr _lidar_timer;
        void lidar_callback(void);
};

#endif
