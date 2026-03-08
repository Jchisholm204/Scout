/**
 * @file telemetry.cpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2026-02-03
 * @modified Last Modified: 2026-02-03
 *
 * @copyright Copyright (c) 2026
 */

#include "telemetry/telemetry.hpp"

#include <arpa/inet.h>
#include <cfloat>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

Telemetry::Telemetry() : Node("sim_telemetry") {
    this->declare_parameter("pub_base", "sim");
    // Set to the Jetson SLAM refresh rate
    this->declare_parameter("pub_rate", 50);

    std::string pub_base =
        this->get_parameter("pub_base").as_string();
    int64_t pub_rate = this->get_parameter("pub_rate").as_int();

    // Sanitize Parameters
    if (pub_rate < 0) {
        throw std::runtime_error("Publish Rate must be greater than 0");
    }

    // Initialize UDP Stream
    struct sockaddr_in servaddr;
    if ((_udp_fp = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("Socket creation Failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(struct sockaddr_in));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(TELEM_PORT);

    if (bind(_udp_fp, (const struct sockaddr*) &servaddr, sizeof(servaddr)) < 0) {
        perror("Bind Failed");
        exit(EXIT_FAILURE);
    }
    int buffer_size =
        sizeof(lf_telemetry_packet_t); // Only hold one packet's worth of data
    setsockopt(_udp_fp, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size));

    _batt_pub =
        this->create_publisher<sensor_msgs::msg::BatteryState>(pub_base + "/batt", 10);
    _imu_pub =
        this->create_publisher<sensor_msgs::msg::Imu>(pub_base + "/imu", 10);
    _pos_pub =
        this->create_publisher<geometry_msgs::msg::Point>(pub_base + "/position", 10);
    _vel_pub =
        this->create_publisher<geometry_msgs::msg::Vector3>(pub_base + "/velocity", 10);
    _time_pub =
        this->create_publisher<std_msgs::msg::Float32>(pub_base + "/time", 10);

    _callback_timer = this->create_wall_timer(std::chrono::milliseconds(pub_rate),
                                              std::bind(&Telemetry::_callback, this));
};

Telemetry::~Telemetry() {
}

void Telemetry::_callback(void) {
    lf_telemetry_packet_t telem_pkt;
    ssize_t n = recvfrom(_udp_fp, &telem_pkt, sizeof(lf_telemetry_packet_t), MSG_DONTWAIT,
                         NULL, NULL);
    if(n != sizeof(lf_telemetry_packet_t)){
        return;
    }
    
    sensor_msgs::msg::BatteryState batt_msg;
    batt_msg.percentage = telem_pkt.batPct;
    batt_msg.voltage = telem_pkt.batVolt;
    _batt_pub->publish(batt_msg);

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "base_link";
    imu_msg.header.stamp = this->now();
    imu_msg.orientation.x = telem_pkt.rotX;
    imu_msg.orientation.y = telem_pkt.rotZ;
    imu_msg.orientation.z = telem_pkt.rotY;
    imu_msg.orientation.w = telem_pkt.rotW;
    _imu_pub->publish(imu_msg);

    geometry_msgs::msg::Point pos_msg;
    pos_msg.x = telem_pkt.posX;
    pos_msg.y = telem_pkt.posZ;
    pos_msg.z = telem_pkt.posY;
    _pos_pub->publish(pos_msg);

    geometry_msgs::msg::Vector3 vel_msg;
    vel_msg.x = telem_pkt.velX;
    vel_msg.y = telem_pkt.velZ;
    vel_msg.z = telem_pkt.velY;
    _vel_pub->publish(vel_msg);

    std_msgs::msg::Float32 time_msg;
    time_msg.data = telem_pkt.timestamp;
    _time_pub->publish(time_msg);
}
