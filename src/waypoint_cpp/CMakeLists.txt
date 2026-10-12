cmake_minimum_required(VERSION 3.8)
project(waypoint_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(waypoint_follower_cpp src/waypoint_follower.cpp)
ament_target_dependencies(waypoint_follower_cpp
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
)

install(TARGETS waypoint_follower_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
