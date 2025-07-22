cmake_minimum_required(VERSION 3.5)
project(io_stall_controller)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  PROGRAMS
    io_stall_controller/io_stall.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
