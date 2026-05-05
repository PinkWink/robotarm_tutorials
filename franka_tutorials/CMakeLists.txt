cmake_minimum_required(VERSION 3.5)
project(franka_tutorials)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
