cmake_minimum_required(VERSION 3.8)
project(circular_trajectory_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(px4_msgs REQUIRED) # <--- 新增的依赖项

# Circular trajectory planner node
add_executable(circular_trajectory_planner_node
  src/circular_trajectory_planner_node.cpp
)

ament_target_dependencies(circular_trajectory_planner_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  px4_msgs # <--- 新增的依赖项
)

# Install
install(TARGETS
  circular_trajectory_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()