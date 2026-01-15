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

int check = 1;
void Driver::_ls_front_callback(const sensor_msgs::msg::LaserScan& ls) {
    if (check == 0) {
        check = 1;
        return;
    }
    check == 0;
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
    _ls_front.range_max = 50;
    _ls_front.range_min = 0;
    _ls_front.angle_increment = ls.angle_increment;
    _ls_front.angle_max = ls.angle_max;
    _ls_front.angle_min = ls.angle_min;
}

void Driver::_ls_vertical_callback(const sensor_msgs::msg::LaserScan& ls) {
    // _usb_send_ls(eLidarVertical, ls);
}
void Driver::_lidar_callback(void) {
    // _ls_front.header.stamp = this->now();
    // _ls_front.header.frame_id = "base_link";
    // _ls_front.angle_min = 0.0f;
    // _ls_front.angle_max = 2.0f * M_PI;
    // _ls_front.range_min = 0.1f;
    // _ls_front.range_max = 50.0f;
    //
    //
    // for (int i = 0; i < 180; i++) {
    //     _ls_front.ranges.push_back(5.0f);
    //     _ls_front.intensities.push_back(1.0f);
    // }
    // // CRITICAL: Calculate increment based on the points we actually got
    // _ls_front.angle_increment =
    //     (_ls_front.angle_max - _ls_front.angle_min) / (float) _ls_front.ranges.size();
    //
    // // Double check sizes match
    // if (_ls_front.ranges.size() == _ls_front.intensities.size()) {
    //     _ls_front_pub->publish(_ls_front);
    //     _ls_vertical_pub->publish(_ls_front);
    //     RCLCPP_INFO(this->get_logger(), "Published %zu points",
    //     _ls_front.ranges.size()); _ls_front.intensities.clear();
    //     _ls_front.ranges.clear();
    // }
    //
    // return;
    struct udev_pkt_lidar ldr_pkt;

    // Drain the USB buffer
    while (_usb_recv_ls(ldr_pkt) == sizeof(struct udev_pkt_lidar)) {
        ldr_data_stale = false;

        // 1. Start of Frame
        if (ldr_pkt.hdr.sequence == 0) {
            _ls_front.ranges.clear();
            _ls_front.intensities.clear();
            _ls_front_count = 0;
        }

        // 2. Data Assembly
        // Verify we are actually getting data in the packet
        if (_ls_front_count == ldr_pkt.hdr.sequence) {
            for (int i = 0; i < ldr_pkt.hdr.len; i++) {
                _ls_front.ranges.push_back(ldr_pkt.distances[i] / 4000.0f);
                _ls_front.intensities.push_back(1.0f);
            }
            _ls_front_count++;
        } else {
            // We missed a packet! The whole scan is now invalid/misaligned.
            // Stop assembling and wait for the next sequence 0.
            _ls_front_count = -1;
            _ls_front.ranges.clear();
            _ls_front.intensities.clear();
        }

        // 3. Completion and Publication
        if (ldr_pkt.hdr.sequence == UDEV_SEQ_MAX - 1) {
            if (!_ls_front.ranges.empty()) {
                // REQUIRED METADATA - RViz needs these BEFORE it can count points
                _ls_front.header.stamp = this->now();
                _ls_front.header.frame_id = "base_link";
                _ls_front.angle_min = 0.0f;
                _ls_front.angle_max = 2.0f * M_PI;
                _ls_front.range_min = 0.1f;
                _ls_front.range_max = 50.0f;

                // CRITICAL: Calculate increment based on the points we actually got
                _ls_front.angle_increment = (_ls_front.angle_max - _ls_front.angle_min) /
                                            (float) _ls_front.ranges.size();

                bool front_null = true;
                for (size_t i = 0; i < _ls_front.ranges.size(); i++) {
                    if (_ls_front.ranges[i] < 40.0f && std::isfinite(_ls_front.ranges[i]))
                        front_null = false;
                }

                // Double check sizes match
                if (_ls_front.ranges.size() == _ls_front.intensities.size() &&
                    !front_null) {
                    _ls_front_pub->publish(_ls_front);
                    _ls_vertical_pub->publish(_ls_front);
                    RCLCPP_INFO(this->get_logger(), "Published %zu points",
                                _ls_front.ranges.size());
                }
            }

            // Set to -1 so we don't append until we see sequence 0 again
            _ls_front_count = -1;
            return; // Exit the while loop after publishing a full frame
        }
    }
}
