cmake_minimum_required(VERSION 3.5)
project(cpu_hog_controller)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY cpu_hog_controller
  DESTINATION lib/python3.10/site-packages
)

ament_package()
