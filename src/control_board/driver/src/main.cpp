#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include "driver/driver.hpp"

#include <unistd.h>
#include <stdbool.h>
#include <memory.h>
#include <libusb-1.0/libusb.h>


int main(int argc, char **argv) {
    printf("Hello World\n");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Driver>());
    rclcpp::shutdown();
    return 0;
}
