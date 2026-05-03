## ── CMakeLists.txt ──────────────────────────────────────────────────────────

cmake_minimum_required(VERSION 3.16)
project(cynlr_bt_manager)

# Require C++17 (needed by BT.CPP v4)
set(CMAKE_CXX_STANDARD 17)

# ── Find packages ─────────────────────────────────────────────────────────────
find_package(ament_cmake    REQUIRED)
find_package(rclcpp         REQUIRED)
find_package(rclcpp_action  REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(behaviortree_cpp REQUIRED)   # BT.CPP v4 apt package name
find_package(std_msgs       REQUIRED)
find_package(std_srvs       REQUIRED)
find_package(nav2_msgs      REQUIRED)
find_package(ament_index_cpp REQUIRED)  # add this near other find_package lines
find_package(cynlr_arm_interfaces REQUIRED)
add_executable(cynlr_bt_manager
  src/cynlr_bt_manager_main.cpp
  bt_action_nodes/bt_movej.cpp
)
target_include_directories(cynlr_bt_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(cynlr_bt_manager
  rclcpp rclcpp_action behaviortree_cpp behaviortree_ros2 ament_index_cpp cynlr_arm_interfaces)


# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS cynlr_bt_manager
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include
)
install(FILES src/tree.xml
  DESTINATION share/${PROJECT_NAME}
)
ament_package()

