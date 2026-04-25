cmake_minimum_required(VERSION 3.15)
project(cynlr_arm_core VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
FetchContent_Declare(
    tl_expected
    GIT_REPOSITORY https://github.com/TartanLlama/expected.git
    GIT_TAG        v1.1.0
    GIT_SHALLOW    TRUE
)
set(EXPECTED_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(tl_expected)

find_package(spdlog REQUIRED)
find_package(flexiv_rdk REQUIRED)

# --- Platform-conditional library type ---
# Linux: SHARED with symbol isolation (hide flexiv_rdk's fastcdr/fastdds from ROS2)
# Windows: STATIC (no ROS2, no conflict)
if(UNIX AND NOT APPLE)
    set(BUILD_SHARED_LIBS ON)
else()
    set(BUILD_SHARED_LIBS OFF)
endif()

set(SOURCES
    src/arm_factory.cpp
    src/sim/sim_arm.cpp
    src/flexiv/flexiv_arm.cpp
)

add_library(cynlr_arm_core ${SOURCES})
add_library(cynlr::arm_core ALIAS cynlr_arm_core)

target_compile_definitions(cynlr_arm_core PRIVATE CYNLR_ARM_CORE_BUILDING)
if(NOT BUILD_SHARED_LIBS)
    target_compile_definitions(cynlr_arm_core PUBLIC CYNLR_ARM_CORE_STATIC)
endif()

target_include_directories(cynlr_arm_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${tl_expected_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(cynlr_arm_core
    PUBLIC
        spdlog::spdlog
    PRIVATE
        flexiv::flexiv_rdk
)

target_compile_features(cynlr_arm_core PUBLIC cxx_std_20)

if(MSVC)
    target_compile_options(cynlr_arm_core PRIVATE /W4)
else()
    target_compile_options(cynlr_arm_core PRIVATE -Wall -Wextra -Wpedantic)
    set_target_properties(cynlr_arm_core PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN ON
    )
    # Hide all symbols from statically absorbed archives (flexiv_rdk, fastcdr,
    # fastdds, boost, etc.) so they don't collide with ROS2's own copies.
    if(BUILD_SHARED_LIBS)
        target_link_options(cynlr_arm_core PRIVATE
            -Wl,--exclude-libs,ALL
            -Wl,-Bsymbolic
            -Wl,--no-undefined
        )
    endif()
endif()


option(BUILD_TESTING "Build unit tests" ON)
if(BUILD_TESTING)
    enable_testing()
    find_package(GTest REQUIRED)

    add_executable(arm_core_tests
        tests/unit/test_factory.cpp
        tests/unit/test_sim_arm.cpp
        tests/unit/test_main.cpp
    )

    target_include_directories(arm_core_tests
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src   # access sim/sim_arm.hpp in tests
    )

    target_link_libraries(arm_core_tests
        PRIVATE
            cynlr::arm_core
            GTest::gtest
    )

    add_test(NAME arm_core_tests COMMAND $<TARGET_FILE:arm_core_tests>)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS cynlr_arm_core
    EXPORT cynlr_arm_coreTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY ${tl_expected_SOURCE_DIR}/include/tl
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT cynlr_arm_coreTargets
    FILE cynlr_arm_coreTargets.cmake
    NAMESPACE cynlr::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cynlr_arm_core
)

configure_package_config_file(
    cmake/cynlr_arm_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cynlr_arm_coreConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cynlr_arm_core
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/cynlr_arm_coreConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/cynlr_arm_coreConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/cynlr_arm_coreConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/cynlr_arm_core
)
