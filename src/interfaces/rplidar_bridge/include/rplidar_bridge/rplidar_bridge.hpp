/**
 * @file rplidar_bridge.hpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief 
 * @version 0.1
 * @date Created: 2025-09-18
 * @modified Last Modified: 2025-09-18
 *
 * @copyright Copyright (c) 2025
 */

#ifndef _RPLIDAR_BRIDGE_HPP_
#define _RPLIDAR_BRIDGE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rplidar.h"
#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include "sensor_msgs/msg/laser_scan.hpp"

class RPLIDARBridge : public rclcpp::Node {
    public:
        RPLIDARBridge();
        ~RPLIDARBridge();
    private:
        sl::ILidarDriver *_lidar;
        sl_lidar_response_device_info_t _lidar_info;
        std::string _frame_id;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _ls_pub;
        void _start(bool on = true, int mode = 0);
        rclcpp::TimerBase::SharedPtr _lidar_timer;
        void lidar_callback(void);
};

#endif
