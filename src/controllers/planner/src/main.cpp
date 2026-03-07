#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "planner/planner.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Planner>());
    rclcpp::shutdown();
    return 0;
}
