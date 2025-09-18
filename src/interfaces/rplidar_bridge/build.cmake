# CMake File for RpLiDAR Bridge
# Jacob Chisholm (https://Jchisholm204.github.io)
# version 0.1
# 2025-09-18
# All paths must be prefixed with ${BASE_DIR} instead of ${CMAKE_CURRENT_SOURCE_DIR}
set(NODE rplidar_bridge)
set(BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${NODE})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Ensure the required packages are avaliable
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# RpLiDAR
set(RPLIDAR_SDK_PATH ${BASE_DIR}/rplidar_sdk/sdk)
FILE(GLOB RPLIDAR_SDK_SRC
    "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/dataunpacker/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/dataunpacker/unpacker/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
    "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

# Gather all sources for this project
file(GLOB_RECURSE PROJECT_SOURCES FOLLOW_SYMLINKS
    ${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/libmsp/src/*.cpp
)

# Setup Library Dependencies
set(${NODE}_DEPS rclcpp std_msgs sensor_msgs)

# Add to include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/include
    ${RPLIDAR_SDK_PATH}/include
    ${RPLIDAR_SDK_PATH}/src
    ${Boost_INCLUDE_DIRS}
)

# Add this node as an executable 
add_executable(${NODE}
    ${PROJECT_SOURCES}
    ${RPLIDAR_SDK_SRC}
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

# Exports
ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/include)
ament_export_targets(export_${NODE} HAS_LIBRARY_TARGET)
ament_export_dependencies(${${NODE}_DEPS})


