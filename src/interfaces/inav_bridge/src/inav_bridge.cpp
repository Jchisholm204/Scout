/**
 * @file inav_bridge.cpp
 * @author Jacob Chisholm (https://Jchisholm204.github.io)
 * @brief
 * @version 0.1
 * @date Created: 2025-09-17
 * @modified Last Modified: 2025-09-17
 *
 * @copyright Copyright (c) 2025
 */

#include "inav_bridge/inav_bridge.hpp"

#include "FlightController.hpp"

#include <unistd.h>

INAVBridge::INAVBridge() : Node("inav_bridge"), _fall_time(5, 0), _wd_maxtime(100, 0) {
    this->declare_parameter("fc_path", "/dev/ttyACM0");
    this->declare_parameter("fc_baud", 115200);
    this->declare_parameter("wd_timeout", 500);
    this->declare_parameter("pub_prefix", "inav");
    this->declare_parameter("movement_topic", "movement");
    this->declare_parameter("arm_topic", "armed");
    this->declare_parameter("fall_time", 5);
    this->declare_parameter("fall_speed", 1500);

    // FC Path/Baud
    std::string fc_path = this->get_parameter("fc_path").as_string();
    int64_t fc_baud = this->get_parameter("fc_baud").as_int();
    // Setup watch dog max timeout
    int64_t wd_timeout = this->get_parameter("wd_timeout").as_int();
    // Pub/Sub prefix
    std::string rtx_pre = this->get_parameter("pub_prefix").as_string();
    // Movement Topic Name
    std::string move_name = this->get_parameter("movement_topic").as_string();
    std::string arm_name = this->get_parameter("arm_topic").as_string();
    _fall_time =
        rclcpp::Duration::from_seconds(this->get_parameter("fall_time").as_int());
    _fall_speed = this->get_parameter("fall_speed").as_int();

    // Parameter Sanitization
    if (access(fc_path.c_str(), F_OK) == -1) {
        throw std::runtime_error("Flight Controller path does not exist.");
    }
    if (fc_baud <= 0) {
        throw std::runtime_error("fc_baud must be > 0");
    }
    if (_fall_speed > 2000) {
        throw std::runtime_error("fall_speed must be less than 2000");
    }
    if (_fall_speed < 0) {
        throw std::runtime_error("fall_speed must be greater than or equal to 0");
    }

    // Watchdog setup
    this->_wd_timer = this->create_wall_timer(std::chrono::milliseconds(wd_timeout / 2),
                                              std::bind(&INAVBridge::wd_callback, this));
    this->_wd_maxtime =
        rclcpp::Duration::from_nanoseconds(static_cast<int64_t>(wd_timeout * 1000000));
    this->_wd_checkin = this->get_clock()->now();

    // INav MSP (MultiWii Serial Protocol) link Setup
    _fc.setLoggingLevel(msp::client::LoggingLevel::SILENT);
    _fc.connect(fc_path, fc_baud);
    _fc.subscribe<msp::msg::RawImu>(
        std::bind(&INAVBridge::on_imu, this, std::placeholders::_1), 0.1);
    _fc.subscribe<msp::msg::Motor>(
        std::bind(&INAVBridge::on_motor, this, std::placeholders::_1), 0.1);
    _fc.subscribe<msp::msg::Attitude>(
        std::bind(&INAVBridge::on_attitude, this, std::placeholders::_1), 0.1);
    _fc.subscribe<msp::msg::Altitude>(
        std::bind(&INAVBridge::on_altitude, this, std::placeholders::_1), 0.1);
    _fc.subscribe<msp::msg::Analog>(
        std::bind(&INAVBridge::on_analog, this, std::placeholders::_1), 0.1);
    _fc.subscribe<msp::msg::DebugMessage>(
        std::bind(&INAVBridge::on_debug_msg, this, std::placeholders::_1), 0.1);

    // Setup FC Mode
    fcu::FlightMode mode;
    mode.primary = fcu::FlightMode::PRIMARY_MODE::ANGLE;
    mode.secondary = fcu::FlightMode::SECONDARY_MODE::NAV_ALTHOLD;
    mode.modifier = fcu::FlightMode::MODIFIER::NONE;
    _fc.setFlightMode(mode);

    // Create ROS Publishers
    this->_imu_pub =
        this->create_publisher<geometry_msgs::msg::Vector3>(rtx_pre + "/imu", 10);
    this->_gyro_pub =
        this->create_publisher<geometry_msgs::msg::Vector3>(rtx_pre + "/gyro", 10);
    this->_orientation_pub =
        this->create_publisher<geometry_msgs::msg::Quaternion>(rtx_pre + "/quat", 10);
    this->_battery_pub =
        this->create_publisher<sensor_msgs::msg::BatteryState>(rtx_pre + "/batt", 10);
    this->_motor_pub =
        this->create_publisher<std_msgs::msg::Float32>(rtx_pre + "/motor_speed", 10);
    this->_altitude_pub =
        this->create_publisher<std_msgs::msg::Float32>(rtx_pre + "/altitude", 10);
    this->_rssi_pub =
        this->create_publisher<std_msgs::msg::Float32>(rtx_pre + "/rssi", 10);

    // Create ROS Subscriptions
    this->_movement_sub = this->create_subscription<geometry_msgs::msg::Quaternion>(
        rtx_pre + move_name, 10,
        std::bind(&INAVBridge::movement_callback, this, std::placeholders::_1));
    this->_arming_sub = this->create_subscription<std_msgs::msg::Bool>(
        rtx_pre + move_name, 10,
        std::bind(&INAVBridge::arming_callback, this, std::placeholders::_1));
}

void INAVBridge::wd_callback(void) {
    rclcpp::Duration t_diff = this->get_clock()->now() - this->_wd_checkin;
    // Timeout Error - Enter Error Handler
    if (t_diff > _wd_maxtime && _fc.isArmed()) {
        // Stop all subscriptions
        _movement_sub.reset();
        _arming_sub.reset();
        // Setup this timer to run more frequently
        _wd_timer->cancel();
        _wd_timer->reset();
        _wd_timer = this->create_wall_timer(std::chrono::milliseconds(50),
                                            std::bind(&INAVBridge::wd_callback, this));
        if (t_diff > _fall_time)
            _fc.setRc(1000, 1000, 1000, 1000, 0);
        else
            _fc.setRc(1000, 1000, 1000, 1000, _fall_speed);
    }
}

void INAVBridge::on_imu(const msp::msg::RawImu& imu) {
    geometry_msgs::msg::Vector3 msg;
    auto readings = msp::msg::ImuSI(imu, 512.0, 1.0 / 4.096, 0.92f / 10.0f, 9.80665f);
    msg.x = readings.acc[0]();
    msg.y = readings.acc[1]();
    msg.z = readings.acc[2]();
    _imu_pub->publish(msg);
}
void INAVBridge::on_motor(const msp::msg::Motor& mtr) {
    std_msgs::msg::Float32 msg;
    for (int i = 0; i < 4; i++)
        msg.data += mtr.motor[i];
    msg.data /= 4;
    _motor_pub->publish(msg);
}

void INAVBridge::on_attitude(const msp::msg::Attitude& att) {
    geometry_msgs::msg::Vector3 msg;
    geometry_msgs::msg::Quaternion q;

    msg.x = att.pitch();
    msg.y = att.roll();
    msg.z = att.yaw();

    float cy = cos(msg.z * 0.5f);
    float sy = sin(msg.z * 0.5f);
    float cp = cos(msg.x * 0.5f);
    float sp = sin(msg.x * 0.5f);
    float cr = cos(msg.y * 0.5f);
    float sr = sin(msg.y * 0.5f);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    _orientation_pub->publish(q);
    _gyro_pub->publish(msg);
}
void INAVBridge::on_altitude(const msp::msg::Altitude& alt) {
    std_msgs::msg::Float32 msg;
    msg.data = alt.altitude;
    _altitude_pub->publish(msg);
}
void INAVBridge::on_analog(const msp::msg::Analog& alg) {
    sensor_msgs::msg::BatteryState msg;
    msg.voltage = alg.vbat();
    msg.capacity = (int) alg.powerMeterSum();
    msg.current = alg.amperage();
    _battery_pub->publish(msg);
    std_msgs::msg::Float32 rssi;
    rssi.data = alg.rssi();
    _rssi_pub->publish(rssi);
}
void INAVBridge::on_debug_msg(const msp::msg::DebugMessage& dbg) {
    RCLCPP_INFO(this->get_logger(), "%s", dbg.debug_msg().c_str());
}
void INAVBridge::movement_callback(const geometry_msgs::msg::Quaternion& q) {
    if (_fc.isArmed() && _fc.isConnected()) {
        _fc.setRc(q.x, q.y, q.w, q.z, 2000);
    }
    _wd_checkin = this->get_clock()->now();
}
void INAVBridge::arming_callback(const std_msgs::msg::Bool& arm) {
    if (_fc.isConnected()) {
        if (arm.data) {
            _fc.setRc(1000, 1000, 1000, 1000, 2000);
            _wd_checkin = this->get_clock()->now();
        }
        else{
            _fc.setRc(1000, 1000, 1000, 1000, 0);
            _wd_checkin = this->get_clock()->now();
        }
    }
}
