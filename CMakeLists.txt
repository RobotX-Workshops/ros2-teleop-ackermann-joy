cmake_minimum_required(VERSION 3.8)
project(teleop_ackermann_joy)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

# Add the executable
add_executable(teleop_ackermann_joy_node src/teleop_ackermann_joy_node.cpp)

# Link libraries
ament_target_dependencies(teleop_ackermann_joy_node
  rclcpp
  sensor_msgs
  ackermann_msgs
)

# Installation rules
install(TARGETS
  teleop_ackermann_joy_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()