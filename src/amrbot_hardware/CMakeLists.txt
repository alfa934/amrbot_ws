cmake_minimum_required(VERSION 3.8)
project(amrbot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


add_library(serialib STATIC src/serialib.cpp)
target_include_directories(serialib PUBLIC include)

add_executable(serial_node src/serial_node.cpp)
ament_target_dependencies(serial_node rclcpp std_msgs geometry_msgs)
target_link_libraries(serial_node serialib)


add_library(amrbot_hardware_interface SHARED
    src/amrbot_hardware_interface.cpp
)
target_include_directories(amrbot_hardware_interface PUBLIC include)
ament_target_dependencies(amrbot_hardware_interface
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
)
target_link_libraries(amrbot_hardware_interface serialib)

install(TARGETS
    amrbot_hardware_interface
    serial_node
    DESTINATION lib/${PROJECT_NAME}
)


pluginlib_export_plugin_description_file(
    hardware_interface
    plugin_description.xml
)


ament_package()
