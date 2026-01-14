/**
 * @file driver_lidar.cpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2026-01-13
 * @modified Last Modified: 2026-01-13
 *
 * @copyright Copyright (c) 2026
 */

#include "driver/driver.hpp"

int Driver::_usb_send_ls(const enum eCBLidar lid, const sensor_msgs::msg::LaserScan& ls) {
    if (!_lusb_hndl) {
        return _lusb_err;
    }
    struct udev_pkt_lidar pkt;
    pkt.hdr.id = (uint8_t) lid;
    pkt.hdr.sequence = 0;
    pkt.hdr.len = 0;
    for (size_t i = 0; i < ls.ranges.size(); i++) {
        pkt.distances[pkt.hdr.len++] = (uint16_t) (ls.ranges[i] * 4.0f);
        // Lidar Packet points reached
        if (pkt.hdr.len == UDEV_LIDAR_POINTS) {

            int transfered = 0;
            _lusb_err =
                libusb_bulk_transfer(_lusb_hndl, LIDAR_RXD_EP, (uint8_t*) &pkt,
                                     sizeof(struct udev_pkt_lidar), &transfered, 0);
            pkt.hdr.sequence++;
            pkt.hdr.len = 0;
        }
    }
    // Send remaining data
    int transfered = 0;
    _lusb_err = libusb_bulk_transfer(_lusb_hndl, LIDAR_RXD_EP, (uint8_t*) &pkt,
                                     sizeof(struct udev_pkt_lidar), &transfered, 0);
    return _lusb_err;
}

int Driver::_usb_recv_ls(struct udev_pkt_lidar& pkt_ldr) {
    if (!_lusb_hndl) {
        return _lusb_err;
    }
    int transfered = 0;
    _lusb_err = libusb_bulk_transfer(_lusb_hndl, LIDAR_TXD_EP, (uint8_t*) &pkt_ldr,
                                     sizeof(struct udev_pkt_lidar), &transfered, 0);
    if (_lusb_err != 0) {
        return 0;
    }
    return transfered;
}

void Driver::_ls_front_callback(const sensor_msgs::msg::LaserScan& ls) {
    // _usb_send_ls(eLidarFront, ls);

    char str[] = "Hello\nHello\nHello\nHello\nHello\nHello\nHello\nHello\nHello\nHello\n"
                 "Hello\nHello\n"
                 "Hello\nHello\n";
    int transfered = 0;
    if (!_lusb_hndl) {
        return;
    }
    _lusb_err = libusb_bulk_transfer(_lusb_hndl, LIDAR_RXD_EP, (uint8_t*) str,
                                     sizeof(struct udev_pkt_lidar), &transfered, 0);
}

void Driver::_ls_vertical_callback(const sensor_msgs::msg::LaserScan& ls) {
    // _usb_send_ls(eLidarVertical, ls);
}

int k = 0;
;
void Driver::_lidar_callback(void) {
    if (!_usb_connected() || !_lusb_hndl) {
        RCLCPP_ERROR(this->get_logger(), "USB Not Connected");
        if (_usb_reconnect()) {
            RCLCPP_ERROR(this->get_logger(), "USB Reconnect Failed");
            return;
        }
    }
    udev_pkt_lidar pkt;
    if (_usb_recv_ls(pkt) != sizeof(struct udev_pkt_lidar)) {
        if (k) {
            RCLCPP_WARN(this->get_logger(), "Failed to RX");
            k = 0;
        }
        return;
    }
    k = 1;
    RCLCPP_INFO(this->get_logger(), "Got %s", (char*) &pkt);
    // sensor_msgs::msg::LaserScan& ls = _ls_front;
    // size_t& count = _ls_front_count;
    // if (pkt.hdr.id == eLidarVertical) {
    //     ls = _ls_vertical;
    //     count = _ls_vertical_count;
    // }
    // // Expect that the incoming sequence number is the same one on file
    // if (pkt.hdr.sequence != count) {
    //     // Restart the sequence on failure
    //     count = 0;
    //     return;
    // }
    // // Increment the count (happens on both sides)
    // count++;
    //
    // for (int i = 0; i < pkt.hdr.len && i < UDEV_LIDAR_POINTS; i++) {
    //     ls.ranges.push_back(pkt.distances[i] / 4.0f);
    // }
    //
    // if (count == UDEV_SEQ_MAX) {
    //     if (pkt.hdr.id == eLidarFront) {
    //         _ls_front_pub->publish(_ls_front);
    //     } else {
    //         _ls_vertical_pub->publish(_ls_vertical);
    //     }
    //     count = 0;
    // }
}
