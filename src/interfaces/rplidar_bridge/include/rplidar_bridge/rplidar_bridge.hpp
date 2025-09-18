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

class RPLIDARBridge : public rclcpp::Node {
    public:
        RPLIDARBridge();
        ~RPLIDARBridge();
    private:
        rclcpp::TimerBase::SharedPtr _timer;
        sl::ILidarDriver *_lidar;
        void timer_callback(void);
};

#endif
