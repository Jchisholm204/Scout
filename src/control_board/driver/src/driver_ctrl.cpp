/**
 * @file driver_ctrl.cpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2026-01-13
 * @modified Last Modified: 2026-01-13
 *
 * @copyright Copyright (c) 2026
 */

#include "driver/driver.hpp"

int Driver::_usb_send_ctrl(const geometry_msgs::msg::Quaternion& qt) {
    struct udev_pkt_ctrl_tx pkt;
    pkt.vel.x = qt.x;
    pkt.vel.y = qt.y;
    pkt.vel.z = qt.z;
    pkt.vel.w = qt.w;
    int transfered = 0;
    _lusb_err = libusb_bulk_transfer(_lusb_hndl, CTRL_RXD_EP, (uint8_t*) &pkt,
                                     sizeof(struct udev_pkt_ctrl_tx), &transfered, 0);
    if (_lusb_err != 0) {
        return 0;
    }
    return transfered;
}

int Driver::_usb_recv_ctrl(struct udev_pkt_ctrl_rx& pkt) {
    int transfered = 0;
    if (!_lusb_hndl) {
        return _lusb_err;
    }
    _lusb_err = libusb_bulk_transfer(_lusb_hndl, CTRL_TXD_EP, (uint8_t*) &pkt,
                                     sizeof(struct udev_pkt_ctrl_rx), &transfered, 0);
    if (_lusb_err != 0) {
        return 0;
    }
    return transfered;
}

void Driver::_ctrl_callback(void) {
    if (!_usb_connected() || !_lusb_hndl) {
        RCLCPP_ERROR(this->get_logger(), "USB Not Connected");
        if (_usb_reconnect()) {
            RCLCPP_ERROR(this->get_logger(), "USB Reconnect Failed");
            return;
        }
    }
    struct udev_pkt_ctrl_rx pkt_rx;
    if(_usb_recv_ctrl(pkt_rx) != sizeof(pkt_rx)){
        return;
    }

    sensor_msgs::msg::BatteryState bat_msg;
    bat_msg.voltage = (float) pkt_rx.vBatt / 5.0f;

    _battery_pub->publish(bat_msg);

    std_msgs::msg::UInt8 state_msg;
    state_msg.data = pkt_rx.status;

    _status_pub->publish(state_msg);

    std_msgs::msg::UInt8 mode_msg;
    mode_msg.data = pkt_rx.mode;

    _mode_pub->publish(mode_msg);

    geometry_msgs::msg::Quaternion qt;
    qt.x = pkt_rx.vel.x;
    qt.y = pkt_rx.vel.y;
    qt.z = pkt_rx.vel.z;
    qt.w = pkt_rx.vel.w;
    _vel_cmd_pub->publish(qt);
    qt.x = pkt_rx.cv.x;
    qt.y = pkt_rx.cv.y;
    qt.z = pkt_rx.cv.z;
    qt.w = pkt_rx.cv.w;
    _col_cmd_pub->publish(qt);
}

void Driver::_vel_callback(const geometry_msgs::msg::Quaternion& qt) {
    if (!_usb_connected() || !_lusb_hndl) {
        RCLCPP_ERROR(this->get_logger(), "USB Not Connected");
        if (_usb_reconnect()) {
            RCLCPP_ERROR(this->get_logger(), "USB Reconnect Failed");
            return;
        }
    }
    _usb_send_ctrl(qt);
}
