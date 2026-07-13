cmake_minimum_required(VERSION 3.16)
project(octo_planner3d LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PCL REQUIRED COMPONENTS io common)
find_package(OpenMP REQUIRED)
find_package(ament_cmake QUIET)

# 全局启用 OpenMP
if(OpenMP_CXX_FOUND)
  add_compile_options(${OpenMP_CXX_FLAGS})
  link_libraries(${OpenMP_CXX_LIBRARIES})
endif()

# OctoMap 库从源码编译 (third_party/octomap submodule)
add_subdirectory(third_party)
add_subdirectory(octomap)
add_subdirectory(planner)

add_executable(globalPlan main.cpp)
target_link_libraries(globalPlan PRIVATE GlobalPlanner Octomap)

if(ament_cmake_FOUND)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(nav2_msgs REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(nav2_core REQUIRED)
  find_package(nav2_costmap_2d REQUIRED)
  find_package(nav2_util REQUIRED)
  find_package(pluginlib REQUIRED)

  add_executable(octo_planner_rviz_node src/octo_planner_rviz_node.cpp)
  target_link_libraries(octo_planner_rviz_node GlobalPlanner Octomap)
  ament_target_dependencies(
    octo_planner_rviz_node
    rclcpp
    geometry_msgs
    nav_msgs
    std_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

  # Nav2 GlobalPlanner plugin library
  add_library(octo_planner_global_planner SHARED
    planner/src/octo_planner_global_planner.cpp
  )
  target_link_libraries(octo_planner_global_planner GlobalPlanner Octomap)
  ament_target_dependencies(
    octo_planner_global_planner
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    nav_msgs
    nav2_core
    nav2_costmap_2d
    nav2_util
    pluginlib
  )

  install(TARGETS
    globalPlan
    octo_planner_rviz_node
    DESTINATION lib/${PROJECT_NAME}
  )

  install(TARGETS
    octo_planner_global_planner
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(TARGETS
    octomap
    octomath
    DESTINATION lib/${PROJECT_NAME}
  )

  install(DIRECTORY
    launch
    config
    DESTINATION share/${PROJECT_NAME}
  )

  pluginlib_export_plugin_description_file(nav2_core plugin.xml)

  ament_package()
endif()
