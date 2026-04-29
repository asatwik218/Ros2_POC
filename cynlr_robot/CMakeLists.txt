cmake_minimum_required(VERSION 3.15)
project(cynlr_robot)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# cynlr_arm_core is built with cmake and installed to cynlr_install/
# Pass -DCMAKE_PREFIX_PATH=.../cynlr_install to colcon build
find_package(cynlr_arm_core REQUIRED)

add_library(cynlr_robot SHARED
    src/cynlr_robot_interface.cpp
    src/cynlr_arm_registry.cpp
)

target_include_directories(cynlr_robot
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cynlr_robot
    rclcpp
    rclcpp_lifecycle
    hardware_interface
    pluginlib
)

target_link_libraries(cynlr_robot cynlr::cynlr_arm_core)

if(MSVC)
    target_compile_options(cynlr_robot PRIVATE /W4)
else()
    target_compile_options(cynlr_robot PRIVATE -Wall -Wextra -Wpedantic)
endif()

# Embed RPATH so the installed .so can find libcynlr_arm_core.so at runtime
get_target_property(_cynlr_loc cynlr::cynlr_arm_core IMPORTED_LOCATION_RELEASE)
get_filename_component(_cynlr_libdir "${_cynlr_loc}" DIRECTORY)
set_target_properties(cynlr_robot PROPERTIES
    INSTALL_RPATH "${_cynlr_libdir}"
    BUILD_WITH_INSTALL_RPATH TRUE
)

pluginlib_export_plugin_description_file(hardware_interface cynlr_robot_plugin.xml)

install(DIRECTORY include/ DESTINATION include)

install(TARGETS cynlr_robot DESTINATION lib)
ament_export_include_directories(include)
ament_export_libraries(cynlr_robot)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
