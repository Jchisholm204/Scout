#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "localization/localization.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Localization>());
    rclcpp::shutdown();
    return 0;
}
