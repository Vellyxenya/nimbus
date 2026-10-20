cmake_minimum_required(VERSION 3.28)
project(nimbus_autonomy_code)

option(BUILD_WITH_ROS "Build ROS components" ON)
option(BUILD_BENCHMARKS "Build local performance benchmarks" ON)

include(CTest)

add_library(nimbus
    src/nimbus/distance_field.cpp
    src/nimbus/nimbus.cpp
    src/nimbus/occupancy_grid.cpp
    src/nimbus/planning/cem_mpc_planner.cpp
    src/nimbus/planning/motion_model.cpp
    src/nimbus/planning/planner_factory.cpp
    src/nimbus/planning/random_shooting_planner.cpp
    src/nimbus/planning/trajectory_objective.cpp
)
target_include_directories(nimbus PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

if(BUILD_BENCHMARKS)
    add_executable(cem_mpc_benchmark benchmarks/cem_mpc_benchmark.cpp)
    target_link_libraries(cem_mpc_benchmark PRIVATE nimbus)
    add_executable(mapping_benchmark benchmarks/mapping_benchmark.cpp)
    target_link_libraries(mapping_benchmark PRIVATE nimbus)
endif()

if(BUILD_WITH_ROS)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp CONFIG REQUIRED)
    find_package(std_msgs CONFIG REQUIRED)
    find_package(geometry_msgs CONFIG REQUIRED)
    find_package(nav_msgs CONFIG REQUIRED)
    find_package(sensor_msgs CONFIG REQUIRED)
    find_package(tf2_ros CONFIG REQUIRED)
    find_package(visualization_msgs CONFIG REQUIRED)

    add_library(nimbus_ros
        src/nimbus/nimbus_ros.cpp
    )
    target_link_libraries(nimbus_ros PUBLIC
        nimbus
    )
    ament_target_dependencies(nimbus_ros PUBLIC
        rclcpp
        geometry_msgs
        nav_msgs
        sensor_msgs
        std_msgs
        tf2_ros
        visualization_msgs
    )

    add_executable(nimbus_autonomy_node src/nimbus/nimbus_autonomy_node.cpp)
    target_link_libraries(nimbus_autonomy_node PRIVATE nimbus_ros)
endif()

if(BUILD_TESTING)
    find_package(GTest CONFIG REQUIRED)
    include(GoogleTest)
    add_subdirectory(tests)
endif()
