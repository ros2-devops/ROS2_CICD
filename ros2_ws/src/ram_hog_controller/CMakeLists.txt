cmake_minimum_required(VERSION 3.5)
project(ram_hog_controller)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  PROGRAMS
    ram_hog_controller/ram_hog.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
