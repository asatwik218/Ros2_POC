cmake_minimum_required(VERSION 3.15)
project(cynlr_camera)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

add_executable(stereo_camera_node
    src/main.cpp
    src/stereo_camera_node.cpp
)

target_include_directories(stereo_camera_node PRIVATE include)

ament_target_dependencies(stereo_camera_node
    rclcpp
    rclcpp_lifecycle
    sensor_msgs
    image_transport
    camera_info_manager
)

if(MSVC)
    target_compile_options(stereo_camera_node PRIVATE /W4)
else()
    target_compile_options(stereo_camera_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS stereo_camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
