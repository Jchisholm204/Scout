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
    struct udev_pkt_lidar pkt;
    pkt.hdr.id = (uint8_t) lid;
    pkt.hdr.sequence = 0;
    pkt.hdr.len = 0;
    for (size_t i = 0; i < ls.ranges.size(); i++) {
        pkt.distances[pkt.hdr.len++] = ls.ranges[i];
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
    int transfered = 0;
    _lusb_err = libusb_bulk_transfer(_lusb_hndl, LIDAR_TXD_EP, (uint8_t*) &pkt_ldr,
                                     sizeof(struct udev_pkt_lidar), &transfered, 0);
    if (_lusb_err != 0) {
        return 0;
    }
    return transfered;
}

void Driver::_ls_front_callback(const sensor_msgs::msg::LaserScan& ls) {
}

void Driver::_ls_vertical_callback(const sensor_msgs::msg::LaserScan& ls) {
}

void Driver::_lidar_callback(void) {
}
