cmake_minimum_required(VERSION 3.15)
project(cynlr_arm_node)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cynlr_arm_interfaces REQUIRED)
find_package(cynlr_robot REQUIRED)
find_package(controller_manager REQUIRED)
find_package(cynlr_arm_core REQUIRED)

# ---------------------------------------------------------------------------
# cynlr_arm_node — shared library (node class used by cynlr_main)
# ---------------------------------------------------------------------------
add_library(cynlr_arm_node_lib SHARED
    src/cynlr_arm_node.cpp
)
target_include_directories(cynlr_arm_node_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(cynlr_arm_node_lib
    rclcpp rclcpp_action geometry_msgs cynlr_arm_interfaces cynlr_robot
)
target_link_libraries(cynlr_arm_node_lib
    ${cynlr_robot_LIBRARIES}   # provides CynlrArmHandle + CynlrArmRegistry (ament export)
    cynlr::cynlr_arm_core
)

get_target_property(_cynlr_loc cynlr::cynlr_arm_core IMPORTED_LOCATION_RELEASE)
get_filename_component(_cynlr_libdir "${_cynlr_loc}" DIRECTORY)
set_target_properties(cynlr_arm_node_lib PROPERTIES
    INSTALL_RPATH "${_cynlr_libdir}"
    BUILD_WITH_INSTALL_RPATH TRUE
    OUTPUT_NAME cynlr_arm_node
)

# ---------------------------------------------------------------------------
# cynlr_main — container executable: ControllerManager + N CynlrArmNodes
# ---------------------------------------------------------------------------
add_executable(cynlr_main src/cynlr_main.cpp)
ament_target_dependencies(cynlr_main rclcpp controller_manager)
target_link_libraries(cynlr_main cynlr_arm_node_lib)
set_target_properties(cynlr_main PROPERTIES
    INSTALL_RPATH "${_cynlr_libdir}"
    BUILD_WITH_INSTALL_RPATH TRUE
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS cynlr_arm_node_lib DESTINATION lib)
install(TARGETS cynlr_main DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(MSVC)
    target_compile_options(cynlr_arm_node_lib PRIVATE /W4)
else()
    target_compile_options(cynlr_arm_node_lib PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_package()
