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

    this->declare_parameter("publish_base", "cb");
    std::string pub_base = this->get_parameter("publish_base").as_string();
    this->declare_parameter("ctrl_rate", 10);
    int ctrl_rate = this->get_parameter("ctrl_rate").as_int();
    this->declare_parameter("lidar_rate", 50);
    int lidar_rate = this->get_parameter("lidar_rate").as_int();

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

    // BEGIN Lidar Interfaces
    _ls_front_pub =
        this->create_publisher<sensor_msgs::msg::LaserScan>(pub_base + "/out/ls_front",
                                                            10);
    _ls_vertical_pub =
        this->create_publisher<sensor_msgs::msg::LaserScan>(pub_base + "/out/ls_vertical",
                                                            10);
    _ls_front_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "lidar_front_frame", 10,
        std::bind(&Driver::_ls_front_callback, this, std::placeholders::_1));
    _ls_vertical_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "lidar_vertical_frame", 10,
        std::bind(&Driver::_ls_vertical_callback, this, std::placeholders::_1));

    _lidar_timer = this->create_wall_timer(std::chrono::milliseconds(lidar_rate),
                                           std::bind(&Driver::_lidar_callback, this));

    _ls_front.ranges.resize(180);
    _ls_front.intensities.resize(180);
    for (int i = 0; i < 180; i++) {
        _ls_front.ranges[i] = 2.0f;
        _ls_front.intensities[i] = 1.0f;
    }

    _ls_vertical.ranges.resize(180);
    _ls_vertical.intensities.resize(180);
    for (int i = 0; i < 180; i++) {
        _ls_vertical.ranges[i] = 2.0f;
        _ls_vertical.intensities[i] = 1.0f;
    }

    // END Lidar Interfaces

    // BEGIN Control Interfaces
    _vel_cmd_pub =
        this->create_publisher<geometry_msgs::msg::Quaternion>(pub_base + "/out/vel", 10);
    _battery_pub =
        this->create_publisher<sensor_msgs::msg::BatteryState>(pub_base + "/out/battery",
                                                               10);
    _mode_pub = this->create_publisher<std_msgs::msg::UInt8>(pub_base + "/out/mode", 10);
    _status_pub =
        this->create_publisher<std_msgs::msg::UInt8>(pub_base + "/out/status", 10);

    _ctrl_timer = this->create_wall_timer(std::chrono::milliseconds(ctrl_rate),
                                          std::bind(&Driver::_ctrl_callback, this));
    // END Control Interfaces
}

Driver::~Driver() {
    libusb_release_interface(_lusb_hndl, CTRL_DATA_INUM);
    libusb_release_interface(_lusb_hndl, LIDAR_DATA_INUM);
    libusb_close(_lusb_hndl);
    libusb_exit(_lusb_ctx);
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
