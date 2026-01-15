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
#include "usb_lidar.h"

int Driver::_usb_send_ls(struct udev_pkt_lidar& pkt_ldr) {
    if (!_lusb_hndl) {
        return _lusb_err;
    }
    // Send remaining data
    int transfered = 0;
    _lusb_err = libusb_bulk_transfer(_lusb_hndl, LIDAR_RXD_EP, (uint8_t*) &pkt_ldr,
                                     sizeof(struct udev_pkt_lidar), &transfered, 1);
    return _lusb_err;
}

int Driver::_usb_recv_ls(struct udev_pkt_lidar& pkt_ldr) {
    if (!_lusb_hndl) {
        return _lusb_err;
    }
    int transfered = 0;
    _lusb_err = libusb_bulk_transfer(_lusb_hndl, LIDAR_TXD_EP, (uint8_t*) &pkt_ldr,
                                     sizeof(struct udev_pkt_lidar), &transfered, 1);
    if (_lusb_err != 0) {
        return 0;
    }
    return transfered;
}

void Driver::_ls_front_callback(const sensor_msgs::msg::LaserScan& ls) {
    udev_pkt_lidar ldr_pkt;
    ldr_pkt.hdr.id = eLidarFront;
    ldr_pkt.hdr.sequence = 0;
    size_t total_len = 0;
    // Only accept lidar packets with the correct number of points
    if (ls.ranges.size() != UDEV_LIDAR_RANGE) {
        return;
    }
    for (int seq = 0; seq < UDEV_SEQ_MAX; seq++) {
        ldr_pkt.hdr.sequence = seq;
        int i = 0;
        for (; i < UDEV_LIDAR_POINTS && total_len < UDEV_LIDAR_RANGE; i++) {
            float d = ls.ranges[total_len];
            if (d < 0.1 || !std::isfinite(d) || d > 45.0f)
                ldr_pkt.distances[i] = 0;
            else
                ldr_pkt.distances[i] = (uint16_t) (d * 4000.0f);
            total_len++;
        }
        ldr_pkt.hdr.len = i;
        _usb_send_ls(ldr_pkt);
    }
}

void Driver::_ls_vertical_callback(const sensor_msgs::msg::LaserScan& ls) {
    udev_pkt_lidar ldr_pkt;
    ldr_pkt.hdr.id = eLidarVertical;
    ldr_pkt.hdr.sequence = 0;
    size_t total_len = 0;
    // Only accept lidar packets with the correct number of points
    if (ls.ranges.size() != UDEV_LIDAR_RANGE) {
        return;
    }
    for (int seq = 0; seq < UDEV_SEQ_MAX; seq++) {
        ldr_pkt.hdr.sequence = seq;
        int i = 0;
        for (; i < UDEV_LIDAR_POINTS && total_len < UDEV_LIDAR_RANGE; i++) {
            float d = ls.ranges[total_len];
            if (d < 0.1 || !std::isfinite(d) || d > 45.0f)
                ldr_pkt.distances[i] = 0;
            else
                ldr_pkt.distances[i] = (uint16_t) (d * 4000.0f);
            total_len++;
        }
        ldr_pkt.hdr.len = i;
        _usb_send_ls(ldr_pkt);
    }
}
void Driver::_lidar_callback(void) {
    struct udev_pkt_lidar ldr_pkt;
    // Ignore invalid packets/Zero length packetsw
    while (_usb_recv_ls(ldr_pkt) == sizeof(struct udev_pkt_lidar)) {
        sensor_msgs::msg::LaserScan* ls = &_ls_front;
        if (ldr_pkt.hdr.id == eLidarVertical) {
            ls = &_ls_vertical;
        }
        ls->angle_min = 0.0f;
        ls->angle_max = 2.0f * M_PI;
        ls->range_min = 0.1f;
        ls->range_max = 50.0f;
        ls->angle_increment = 2 * M_PI / 180.0f;
        _ls_front.header.frame_id = "lidar_front_frame";
        _ls_vertical.header.frame_id = "lidar_vertical_frame";
        ls->header.stamp = this->now();
        ls->intensities.resize(180);
        ls->ranges.resize(180);

        bool scan_null = true;
        int global_index = ldr_pkt.hdr.sequence * UDEV_LIDAR_POINTS;
        for (int i = 0; i < ldr_pkt.hdr.len; i++) {
            float distance = ldr_pkt.distances[i] / 4000.0f;
            if (distance < 45)
                scan_null = false;
            // ls.ranges.push_back(distance);
            // ls.intensities.push_back(1.0f);
            ls->intensities[global_index + i] = 1.0f;
            ls->ranges[global_index + i] = distance;
        }
        if (!scan_null) {
            if (ldr_pkt.hdr.id == eLidarVertical) {
                _ls_vertical_pub->publish(_ls_vertical);

            } else {
                _ls_front_pub->publish(_ls_front);
            }
        }
    }
    RCLCPP_INFO(this->get_logger(), "Published Points");
}
