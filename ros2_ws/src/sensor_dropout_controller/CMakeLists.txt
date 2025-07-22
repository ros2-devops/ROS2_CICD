cmake_minimum_required(VERSION 3.5)
project(sensor_dropout_controller)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  PROGRAMS
    sensor_dropout_controller/sensor_dropout.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
