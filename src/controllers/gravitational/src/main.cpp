#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "gravitational/gravitational.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Gravitational>());
    rclcpp::shutdown();
    return 0;
}
