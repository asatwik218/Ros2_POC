cmake_minimum_required(VERSION 3.15)
project(cynlr_arm_controllers)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cynlr_arm_interfaces REQUIRED)
find_package(cynlr_arm_core REQUIRED)

# ---------------------------------------------------------------------------
# Helper: RPATH setup
# ---------------------------------------------------------------------------
get_target_property(_cynlr_loc cynlr::cynlr_arm_core IMPORTED_LOCATION_RELEASE)
get_filename_component(_cynlr_libdir "${_cynlr_loc}" DIRECTORY)

# ---------------------------------------------------------------------------
# cynlr_direct_command_controller  (escape hatch #1)
# ---------------------------------------------------------------------------
add_library(cynlr_direct_command_controller SHARED
    cynlr_direct_command_controller/src/cynlr_direct_command_controller.cpp
)
target_include_directories(cynlr_direct_command_controller PRIVATE
    cynlr_direct_command_controller/include
)
ament_target_dependencies(cynlr_direct_command_controller
    rclcpp rclcpp_lifecycle controller_interface hardware_interface
    pluginlib realtime_tools cynlr_arm_interfaces
)

# ---------------------------------------------------------------------------
# cynlr_cartesian_controller  (escape hatch #2)
# ---------------------------------------------------------------------------
add_library(cynlr_cartesian_controller SHARED
    cynlr_cartesian_controller/src/cynlr_cartesian_controller.cpp
)
target_include_directories(cynlr_cartesian_controller PRIVATE
    cynlr_cartesian_controller/include
)
ament_target_dependencies(cynlr_cartesian_controller
    rclcpp rclcpp_lifecycle controller_interface hardware_interface
    pluginlib realtime_tools cynlr_arm_interfaces
)

# ---------------------------------------------------------------------------
# Plugin registration
# ---------------------------------------------------------------------------
pluginlib_export_plugin_description_file(controller_interface cynlr_controllers_plugin.xml)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS
    cynlr_direct_command_controller
    cynlr_cartesian_controller
    DESTINATION lib
)

if(MSVC)
    target_compile_options(cynlr_direct_command_controller PRIVATE /W4)
else()
    foreach(_tgt cynlr_direct_command_controller cynlr_cartesian_controller)
        target_compile_options(${_tgt} PRIVATE -Wall -Wextra -Wpedantic)
    endforeach()
endif()

ament_package()
