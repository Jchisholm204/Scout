/**
 * @file rplidar_bridge.cpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2025-09-18
 * @modified Last Modified: 2025-09-18
 *
 * @copyright Copyright (c) 2025
 */

#include "lidarstreams/lidarstreams.hpp"

#include <arpa/inet.h>
#include <cfloat>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

LidarStreams::LidarStreams() : Node("lidarstreams") {
    this->declare_parameter("pub_prefix", "sim");
    this->declare_parameter("lidar_front_name", "/ls_front");
    this->declare_parameter("lidar_vertical_name", "/ls_vertical");
    this->declare_parameter("pub_rate", 50);

    std::string pub_prefix = this->get_parameter("pub_prefix").as_string();
    std::string lidar_front_name = this->get_parameter("lidar_front_name").as_string();
    std::string lidar_vertical_name =
        this->get_parameter("lidar_vertical_name").as_string();
    int64_t pub_rate = this->get_parameter("pub_rate").as_int();

    // Sanitize Parameters
    if (pub_rate < 0) {
        throw std::runtime_error("Publish Rate must be greater than 0");
    }

    // Initialize LiDAR UDP Stream
    struct sockaddr_in servaddr;
    if ((_udp_fp = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation Failed");
        exit(EXIT_FAILURE);
    }
    memset(&servaddr, 0, sizeof(struct sockaddr_in));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    if (bind(_udp_fp, (const struct sockaddr*) &servaddr, sizeof(servaddr)) < 0) {
        perror("Bind Failed");
        exit(EXIT_FAILURE);
    }
    int buffer_size = sizeof(struct lidar_packet); // Only hold one packet's worth of data
    setsockopt(_udp_fp, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size));

    _ls_front_pub =
        this->create_publisher<sensor_msgs::msg::LaserScan>(pub_prefix + lidar_front_name,
                                                            10);
    _ls_vertical_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(
        pub_prefix + lidar_vertical_name, 10);

    _lidar_timer =
        this->create_wall_timer(std::chrono::milliseconds(pub_rate),
                                std::bind(&LidarStreams::lidar_callback, this));
};

LidarStreams::~LidarStreams() {
}

void LidarStreams::lidar_callback(void) {
    // RCLCPP_INFO(this->get_logger(), "Lidar Callback!\n");

    struct lidar_packet ldr_pkt;
    ssize_t n = recvfrom(_udp_fp, &ldr_pkt, sizeof(struct lidar_packet), MSG_DONTWAIT,
                         NULL, NULL);

    if (n < 0 || (size_t) n < sizeof(struct lidar_packet)) {
        RCLCPP_WARN(this->get_logger(), "Invalid/No Data");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "F: %2.2f T: %2.2f B: %2.2f", ldr_pkt.front[0],
                ldr_pkt.vertical[0], ldr_pkt.vertical[N_POINTS / 2]);

    sensor_msgs::msg::LaserScan msg_front;
    msg_front.angle_max = 2 * M_PI;
    msg_front.angle_min = 0;
    msg_front.range_max = 50.0;
    msg_front.range_min = 0.5;
    msg_front.header.stamp = this->now();
    msg_front.ranges.resize(N_POINTS);
    msg_front.intensities.resize(N_POINTS);

    sensor_msgs::msg::LaserScan msg_vertical;
    msg_vertical.angle_max = 2 * M_PI;
    msg_vertical.angle_min = 0;
    msg_vertical.range_max = 50.0;
    msg_vertical.range_min = 0.5;
    msg_vertical.header.stamp = this->now();
    msg_vertical.ranges.resize(N_POINTS);
    msg_vertical.intensities.resize(N_POINTS);

    bool front_null = true;
    bool vertical_null = true;

    for (size_t i = 0; i < N_POINTS; i++) {
        msg_front.ranges[i] = (ldr_pkt.front[(N_POINTS - 1) - i]);
        if(msg_front.ranges[i] < msg_front.range_min){
            msg_front.ranges[i] = std::numeric_limits<float>::infinity();
        }
        msg_front.intensities[i] = (1);
        msg_vertical.ranges[i] = (ldr_pkt.vertical[i]);
        if(msg_vertical.ranges[i] < msg_vertical.range_min){
            msg_vertical.ranges[i] = std::numeric_limits<float>::infinity();
        }
        msg_vertical.intensities[i] = (1);
        if (ldr_pkt.front[i] < 45) {
            front_null = false;
        }
        if (ldr_pkt.vertical[i] < 45) {
            vertical_null = false;
        }
    }

    msg_front.angle_increment =
        (msg_front.angle_max - msg_front.angle_min) / (float) (N_POINTS);
    msg_vertical.angle_increment =
        (msg_vertical.angle_max - msg_vertical.angle_min) / (float) (N_POINTS);

    msg_front.header.frame_id = "lidar_front_frame";
    if (!front_null)
        _ls_front_pub->publish(msg_front);
    msg_vertical.header.frame_id = "lidar_vertical_frame";
    if (!vertical_null)
        _ls_vertical_pub->publish(msg_vertical);
}
