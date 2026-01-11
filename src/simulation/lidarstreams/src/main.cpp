#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "lidarstreams/lidarstreams.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarStreams>());
    rclcpp::shutdown();
    std::cout << "Hello World!" << std::endl;
    return 0;
}
