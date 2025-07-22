cmake_minimum_required(VERSION 3.5)
project(slam_node_controller)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  PROGRAMS
    slam_node_controller/slam_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
