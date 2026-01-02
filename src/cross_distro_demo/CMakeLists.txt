cmake_minimum_required(VERSION 3.5)
project(cross_distro_demo)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)

# Include directories
include_directories(include)

# Add executable for the publisher
add_executable(pc_publisher src/cross_distro_demo/pc_publisher.cpp)
ament_target_dependencies(pc_publisher rclcpp custom_interfaces)

# Add executable for the subscriber
add_executable(pi_subscriber src/cross_distro_demo/pi_subscriber.cpp)
ament_target_dependencies(pi_subscriber rclcpp custom_interfaces)

# Install targets
install(TARGETS
  pc_publisher
  pi_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()