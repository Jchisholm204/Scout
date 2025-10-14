cmake_minimum_required(VERSION 3.8)
project(control_board)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(driver/build.cmake)

# Do NOT link the embedded CMakeLists to ROS
#  Colcon is far to stupid to properly understand cross compilation with nano specs (no OS)

ament_package()
