cmake_minimum_required(VERSION 3.5)
project(cpu_hog_controller)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  PROGRAMS
    cpu_hog_controller/setup.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
