set(NODE inav_bridge)

# Ensure the required packages are avaliable
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Ensure executables go into the build folder
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Add the MSP submodule (examples/tests excluded)
add_subdirectory(${NODE}/libmsp EXCLUDE_FROM_ALL)

# Gather sources
file(GLOB_RECURSE PROJECT_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/src/*.cpp
)

# Add executable
add_executable(${NODE} ${PROJECT_SOURCES})


# Link MSP libraries
target_link_libraries(${NODE} mspclient msp_fcu)

# Include directories (use BUILD/INSTALL interface)
target_include_directories(${NODE} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/libmsp/inc/msp>
    $<INSTALL_INTERFACE:include>
)

# Add these flags to your target
target_compile_options(${NODE} PRIVATE
    -fsanitize=address
    -fsanitize=undefined
)
target_link_options(${NODE} PRIVATE
    -fsanitize=address
    -fsanitize=undefined
)


# Link ROS 2 dependencies
ament_target_dependencies(${NODE} rclcpp sensor_msgs std_msgs)

# Install headers
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/include/
    DESTINATION include/${PROJECT_NAME}
)

# Install executable
install(
    TARGETS ${NODE}
    EXPORT export_${NODE}
    DESTINATION lib/${PROJECT_NAME}
)

# Export targets and includes (before final ament_package in parent)
ament_export_include_directories($<INSTALL_INTERFACE:include>)
ament_export_targets(export_${NODE} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs std_msgs)

