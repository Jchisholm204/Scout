# CMake File for INAV Bridge
# Jacob Chisholm (https://Jchisholm204.github.io)
# version 0.1
# 2025-03-23
set(NODE inav_bridge)

# Ensure the required packages are avaliable
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/libmsp/cmake/")
find_package(ASIO REQUIRED)
add_definitions(-DASIO_STANDALONE)
add_definitions(-DASIO_HAS_STD_ADDRESSOF)
add_definitions(-DASIO_HAS_STD_ARRAY)
add_definitions(-DASIO_HAS_CSTDINT)
add_definitions(-DASIO_HAS_STD_SHARED_PTR)
add_definitions(-DASIO_HAS_STD_TYPE_TRAITS)

# Gather all sources for this project
file(GLOB_RECURSE PROJECT_SOURCES FOLLOW_SYMLINKS
    ${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/libmsp/src/*.cpp
)

# Setup Library Dependencies
set(${NODE}_DEPS rclcpp std_msgs sensor_msgs ASIO Threads tf2_ros)

# Add to include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/libmsp/inc/msp
)

# Add this node as an executable 
add_executable(${NODE}
    ${PROJECT_SOURCES}
)

target_include_directories(${NODE} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/include>
    $<INSTALL_INTERFACE:include>
)


# Library dependencies
ament_target_dependencies(${NODE} ${${NODE}_DEPS})


# Install header files
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/include/
    DESTINATION include/${PROJECT_NAME}
)

# Install library files
install(
    TARGETS ${NODE}
    EXPORT export_${NODE}
    DESTINATION lib/${PROJECT_NAME}
)

# Install UDRF
install(
    DIRECTORY udrf
    DESTINATION share/${PROJECT_NAME}
)


# Exports
ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/include)
ament_export_targets(export_${NODE} HAS_LIBRARY_TARGET)
ament_export_dependencies(${${NODE}_DEPS})


