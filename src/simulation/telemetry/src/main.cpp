#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "telemetry/telemetry.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Telemetry>());
    rclcpp::shutdown();
    std::cout << "Hello World!" << std::endl;
    return 0;
}
