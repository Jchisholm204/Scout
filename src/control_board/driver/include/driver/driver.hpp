/**
 * @file driver.hpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2026-01-13
 * @modified Last Modified: 2026-01-13
 *
 * @copyright Copyright (c) 2026
 */

#ifndef _DRIVER_HPP_
#define _DRIVER_HPP_

#include "usb_cb_defs.h"
#include "usb_dev.h"
#include "usb_packet.h"

#include <geometry_msgs/msg/quaternion.hpp>
#include <libusb-1.0/libusb.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/u_int8.hpp>

class Driver : public rclcpp::Node {
  public:
    Driver();
    ~Driver();

  private:
    // USB Device Interfacing
    libusb_context* _lusb_ctx = NULL;
    libusb_device_handle* _lusb_hndl;
    int _lusb_err;
    int _usb_send_ls(struct udev_pkt_lidar& pkt_ldr);
    int _usb_recv_ls(struct udev_pkt_lidar& pkt_ldr);
    int _usb_send_ctrl(const geometry_msgs::msg::Quaternion& qt, enum eCBMode mode);
    int _usb_recv_ctrl(struct udev_pkt_ctrl_rx& pkt);
    int _usb_connected(void);
    int _usb_reconnect(void);

    // LiDAR EP Callbacks
    // Output for running in regular mode
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _ls_front_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr _ls_vertical_pub;
    // Input for running in simulation mode
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _ls_front_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _ls_vertical_sub;
    void _ls_front_callback(const sensor_msgs::msg::LaserScan& ls);
    void _ls_vertical_callback(const sensor_msgs::msg::LaserScan& ls);
    // EP read callback
    rclcpp::TimerBase::SharedPtr _lidar_timer;
    sensor_msgs::msg::LaserScan _ls_front, _ls_vertical;
    int _ls_front_count = 0, _ls_vertical_count = 0;
    bool ldr_data_stale = 1;
    void _lidar_callback(void);

    // Control EP Callbacks
    rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr _vel_cmd_pub;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr _battery_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr _mode_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr _status_pub;
    rclcpp::TimerBase::SharedPtr _ctrl_timer;
    void _ctrl_callback(void);

    // rclcpp::Subscription<std_msgs::msg::UInt8> _mode_sub;
    void _mode_callback(std_msgs::msg::UInt8& new_mode);
    enum eCBMode _new_mode;
    // rclcpp::Subscription<geometry_msgs::msg::Quaternion> _vel_cmd_sub;
    void _vel_callback(const geometry_msgs::msg::Quaternion& qt);
};

#endif
