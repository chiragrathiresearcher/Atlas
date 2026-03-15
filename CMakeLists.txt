cmake_minimum_required(VERSION 3.16)
project(ATLAS_ARVS VERSION 1.0.0 LANGUAGES CXX)

# ─────────────────────────────────────────────────────────────────
#  Compiler settings — safety-critical C++17
# ─────────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Maximum warning level + treat warnings as errors
add_compile_options(
    -Wall -Wextra -Wpedantic -Werror
    -Wshadow -Wnon-virtual-dtor -Wold-style-cast
    -Woverloaded-virtual -Wnull-dereference
    -Wformat=2 -Wduplicated-cond
    # Safety-critical: no undefined behavior
    -fno-delete-null-pointer-checks
    -fstack-protector-strong
)

# Release: optimize for real-time performance
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-O2 -DNDEBUG)
endif()

# ─────────────────────────────────────────────────────────────────
#  Include paths
# ─────────────────────────────────────────────────────────────────
# ATLAS headers
set(ATLAS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp/include)

# ARVS C++ headers (from ARVS repo — adjust path if needed)
# Expected: ../ARVS/cpp/include/
set(ARVS_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ARVS/ARVS-main/cpp/include)

include_directories(
    ${ATLAS_INCLUDE_DIR}
    ${ARVS_INCLUDE_DIR}
)

# ─────────────────────────────────────────────────────────────────
#  ARVS C++ static library (from ARVS repo)
# ─────────────────────────────────────────────────────────────────
set(ARVS_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ARVS/ARVS-main/cpp)

add_library(arvs_cpp STATIC
    ${ARVS_CPP_DIR}/src/axiom_validator.cpp
    ${ARVS_CPP_DIR}/src/safety_gate.cpp
    ${ARVS_CPP_DIR}/src/execution_controller.cpp
    ${ARVS_CPP_DIR}/src/watchdog_timer.cpp
)
target_include_directories(arvs_cpp PUBLIC ${ARVS_INCLUDE_DIR})

# ─────────────────────────────────────────────────────────────────
#  ATLAS flight computer library
# ─────────────────────────────────────────────────────────────────
add_library(atlas_flight STATIC
    src/cpp/src/flight_computer.cpp
    src/cpp/src/imu_driver.cpp
    src/cpp/src/gnss_driver.cpp
    src/cpp/src/actuator_controller.cpp
    src/cpp/src/safety_gate.cpp
)
target_include_directories(atlas_flight PUBLIC ${ATLAS_INCLUDE_DIR} ${ARVS_INCLUDE_DIR})
target_link_libraries(atlas_flight PUBLIC arvs_cpp)

# ─────────────────────────────────────────────────────────────────
#  Flight computer executable
# ─────────────────────────────────────────────────────────────────
add_executable(atlas_fc
    src/cpp/src/main_fc.cpp
)
target_link_libraries(atlas_fc PRIVATE atlas_flight arvs_cpp)

# ─────────────────────────────────────────────────────────────────
#  ROS2 nodes (optional — requires ament_cmake)
# ─────────────────────────────────────────────────────────────────
option(BUILD_ROS2_NODES "Build ROS2 nodes" OFF)

if(BUILD_ROS2_NODES)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

    add_executable(atlas_flight_node
        src/cpp/ros2/flight_computer_node.cpp
    )
    ament_target_dependencies(atlas_flight_node rclcpp std_msgs)
    target_link_libraries(atlas_flight_node atlas_flight)

    add_executable(atlas_safety_node
        src/cpp/ros2/safety_gate_node.cpp
    )
    ament_target_dependencies(atlas_safety_node rclcpp std_msgs)
    target_link_libraries(atlas_safety_node atlas_flight)
endif()

# ─────────────────────────────────────────────────────────────────
#  Tests (C++)
# ─────────────────────────────────────────────────────────────────
option(BUILD_TESTS "Build unit tests" ON)

if(BUILD_TESTS)
    enable_testing()

    add_executable(test_imu_driver       tests/test_imu_driver.cpp)
    add_executable(test_safety_gate      tests/test_safety_gate.cpp)
    add_executable(test_actuator_limits  tests/test_actuator_limits.cpp)
    add_executable(test_arvs_integration tests/test_arvs_integration.cpp)

    foreach(test_exe
        test_imu_driver test_safety_gate
        test_actuator_limits test_arvs_integration)

        target_link_libraries(${test_exe} PRIVATE atlas_flight arvs_cpp)
        add_test(NAME ${test_exe} COMMAND ${test_exe})
    endforeach()
endif()

# ─────────────────────────────────────────────────────────────────
#  Install
# ─────────────────────────────────────────────────────────────────
install(TARGETS atlas_fc DESTINATION bin)
install(DIRECTORY ${ATLAS_INCLUDE_DIR}/ DESTINATION include/atlas)
install(TARGETS atlas_flight arvs_cpp DESTINATION lib)

# ─────────────────────────────────────────────────────────────────
#  Summary
# ─────────────────────────────────────────────────────────────────
message(STATUS "=== ATLAS Build Configuration ===")
message(STATUS "  Build type:     ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard:   C++${CMAKE_CXX_STANDARD}")
message(STATUS "  ATLAS headers:  ${ATLAS_INCLUDE_DIR}")
message(STATUS "  ARVS headers:   ${ARVS_INCLUDE_DIR}")
message(STATUS "  ROS2 nodes:     ${BUILD_ROS2_NODES}")
message(STATUS "  Tests:          ${BUILD_TESTS}")
message(STATUS "=================================")
