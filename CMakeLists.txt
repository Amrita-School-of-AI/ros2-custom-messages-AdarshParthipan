cmake_minimum_required(VERSION 3.8)
project(ros2_custom_msgs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(status_publisher src/status_publisher.cpp)
ament_target_dependencies(status_publisher rclcpp)

# Link generated message headers
rosidl_target_interfaces(status_publisher
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  status_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
