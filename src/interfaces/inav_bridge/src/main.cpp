#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "inav_bridge/inav_bridge.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<INAVBridge>());
    rclcpp::shutdown();
    return 0;
}

