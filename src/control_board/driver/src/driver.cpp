/**
 * @file driver.cpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2026-01-13
 * @modified Last Modified: 2026-01-13
 *
 * @copyright Copyright (c) 2026
 */

#include "driver/driver.hpp"

Driver::Driver() : Node("cb_driver") {

    // Begin Libusb Initialization
    this->_lusb_ctx = NULL;
    this->_lusb_hndl = NULL;
    this->_lusb_err = 0;
    _lusb_err = libusb_init(&_lusb_ctx);
    if (_lusb_err != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to Initialize libusb");
        exit(-1);
    }
    while (!_usb_connected()) {
        RCLCPP_WARN(this->get_logger(), "Waiting for USB device Connection");
        usleep(100000);
    }
    _usb_reconnect();
    if (_lusb_err != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to Initialize USB Device - Exiting");
        libusb_exit(_lusb_ctx);
        exit(_lusb_err);
    }
    // END Libusb Initialization

    _ctrl_timer = this->create_wall_timer(std::chrono::milliseconds(50),
                                          std::bind(&Driver::_ctrl_callback, this));
}

Driver::~Driver() {
    libusb_release_interface(_lusb_hndl, CTRL_DATA_INUM);
    libusb_release_interface(_lusb_hndl, LIDAR_DATA_INUM);
    libusb_close(_lusb_hndl);
    libusb_exit(_lusb_ctx);
}

int Driver::_usb_send_ls(const enum eCBLidar lid, const sensor_msgs::msg::LaserScan& ls) {
}
int Driver::_usb_recv_ls(struct udev_pkt_lidar& pkt_ldr) {
}
int Driver::_usb_send_ctrl(const geometry_msgs::msg::Quaternion& qt, enum eCBMode mode) {
}
int Driver::_usb_recv_ctrl(struct udev_pkt_ctrl_rx& pkt) {
    int transfered = 0;
    _lusb_err = libusb_bulk_transfer(_lusb_hndl, CTRL_TXD_EP, (uint8_t*) &pkt,
                                     sizeof(struct udev_pkt_ctrl_rx), &transfered, 0);
    if (_lusb_err != 0) {
        return 0;
    }
    return transfered;
}
int Driver::_usb_connected(void) {
    libusb_device** devices;
    ssize_t count;
    int found = 0;
    count = libusb_get_device_list(_lusb_ctx, &devices);

    if (count < 0) {
        _lusb_err = count;
        return 0;
    }

    // Iterate through the list of devices
    for (ssize_t i = 0; i < count; i++) {
        libusb_device* device = devices[i];
        struct libusb_device_descriptor desc;

        // Get the device descriptor
        int ret = libusb_get_device_descriptor(device, &desc);
        if (ret < 0) {
            fprintf(stderr, "Error getting device descriptor: %s\n",
                    libusb_error_name(ret));
            continue;
        }

        // Check if the device matches the vendor and product IDs
        if (desc.idVendor == VENDOR_ID && desc.idProduct == DEVICE_ID) {
            found = 1;
            break; // Device found
        }
    }

    // Free the device list
    libusb_free_device_list(devices, 1);

    return found; // 1 if device is connected, 0 otherwise
}
int Driver::_usb_reconnect(void) {
    _lusb_hndl = libusb_open_device_with_vid_pid(_lusb_ctx, VENDOR_ID, DEVICE_ID);
    if (!_lusb_hndl) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open usb device");
        return -1;
    }
    // Detach kernel driver if necessary
    if (libusb_kernel_driver_active(_lusb_hndl, CTRL_DATA_INUM) == 1) {
        _lusb_err = libusb_detach_kernel_driver(_lusb_hndl, CTRL_DATA_INUM);
        if (_lusb_err != 0) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to Detach Kernel Driver on CTRL INUM");
            libusb_close(_lusb_hndl);
            return _lusb_err;
        }
    }
    if (libusb_kernel_driver_active(_lusb_hndl, LIDAR_DATA_INUM) == 1) {
        _lusb_err = libusb_detach_kernel_driver(_lusb_hndl, LIDAR_DATA_INUM);
        if (_lusb_err != 0) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to Detach Kernel Driver on LIDAR INUM");
            libusb_close(_lusb_hndl);
            return _lusb_err;
        }
    }

    // Claim the interfaces
    _lusb_err = libusb_claim_interface(_lusb_hndl, CTRL_DATA_INUM);
    if (_lusb_err != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to claim CTRL interface");
        libusb_close(_lusb_hndl);
        return _lusb_err;
    }

    _lusb_err = libusb_claim_interface(_lusb_hndl, LIDAR_DATA_INUM);
    if (_lusb_err != 0) {
        libusb_close(_lusb_hndl);
        RCLCPP_ERROR(this->get_logger(), "Failed to claim LIDAR interface");
        return _lusb_err;
    }
    return 0;
}

void Driver::_ls_front_callback(const sensor_msgs::msg::LaserScan& ls) {
}
void Driver::_ls_vertical_callback(const sensor_msgs::msg::LaserScan& ls) {
}
void Driver::_lidar_callback(void) {
}
void Driver::_ctrl_callback(void) {
    if (!_usb_connected()) {
        if (_usb_reconnect()) {
            RCLCPP_ERROR(this->get_logger(), "USB Reconnect Failed");
            return;
        }
    }
    struct udev_pkt_ctrl_rx pkt_rx;
    _usb_recv_ctrl(pkt_rx);
    RCLCPP_INFO(this->get_logger(), "USB OK");
    RCLCPP_INFO(this->get_logger(), "%1.3f %1.3f %1.3f %1.3f", pkt_rx.vel.x, pkt_rx.vel.y,
                pkt_rx.vel.z, pkt_rx.vel.w);
}
void Driver::_mode_callback(std_msgs::msg::UInt8& qt) {
}
void Driver::_vel_callback(const geometry_msgs::msg::Quaternion& qt) {
}
