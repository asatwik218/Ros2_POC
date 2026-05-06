cmake_minimum_required(VERSION 3.15)
project(cynlr_arm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

ament_package()
