cmake_minimum_required(VERSION 3.5)
project(custom_interfaces)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CrossDistroMessage.msg"
)

ament_package()