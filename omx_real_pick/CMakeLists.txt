cmake_minimum_required(VERSION 3.8)
project(omx_real_pick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros2_aruco_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

# 1. 제어 노드 (C++)
add_executable(real_pick_node src/real_pick_node.cpp)
ament_target_dependencies(real_pick_node
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_ros_move_group
  geometry_msgs
  control_msgs
  tf2_ros
  tf2_geometry_msgs
  ros2_aruco_interfaces
  sensor_msgs
)

# 2. 설치 설정
install(TARGETS
  real_pick_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/aruco_realsense.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()