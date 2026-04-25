cmake_minimum_required(VERSION 3.15)
project(cynlr_arm_service)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(cynlr_arm_interfaces REQUIRED)

# cynlr_arm_core is built with cmake and installed to cynlr_install/
# Pass -DCMAKE_PREFIX_PATH=.../cynlr_install to colcon build
find_package(cynlr_arm_core REQUIRED)

add_executable(arm_service_node
    src/main.cpp
    src/arm_service_node.cpp
    src/mode_manager.cpp
)

target_include_directories(arm_service_node PRIVATE include)

ament_target_dependencies(arm_service_node
    rclcpp
    rclcpp_lifecycle
    cynlr_arm_interfaces
)

target_link_libraries(arm_service_node cynlr::cynlr_arm_core)

# Embed RPATH so the installed binary can find libcynlr_arm_core.so at runtime
get_target_property(_cynlr_loc cynlr::cynlr_arm_core IMPORTED_LOCATION_RELEASE)
get_filename_component(_cynlr_libdir "${_cynlr_loc}" DIRECTORY)
set_target_properties(arm_service_node PROPERTIES
    INSTALL_RPATH "${_cynlr_libdir}"
    BUILD_WITH_INSTALL_RPATH TRUE
)

if(MSVC)
    target_compile_options(arm_service_node PRIVATE /W4)
else()
    target_compile_options(arm_service_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS arm_service_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Integration tests (Phase 8)
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
    find_package(GTest REQUIRED)
    find_package(launch_testing_ament_cmake REQUIRED)

    # C++ test binary — built but NOT registered as standalone ctest.
    # Invoked by the launch test after arm_service_node is up and active.
    add_executable(test_arm_service_integration
        tests/integration/test_arm_service.cpp
    )
    ament_target_dependencies(test_arm_service_integration
        rclcpp
        cynlr_arm_interfaces
    )
    target_link_libraries(test_arm_service_integration GTest::gtest)
    set_target_properties(test_arm_service_integration PROPERTIES
        INSTALL_RPATH "${_cynlr_libdir}"
        BUILD_WITH_INSTALL_RPATH TRUE
    )
    install(TARGETS test_arm_service_integration DESTINATION lib/${PROJECT_NAME})

    add_launch_test(
        tests/integration/test_arm_service.launch.py
        TARGET test_arm_service_launch
        TIMEOUT 120
    )
endif()

ament_package()
