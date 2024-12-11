cmake_minimum_required(VERSION 3.8)
project(mowing_robot_bt)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include)

# Add the executable
add_executable(task_planner src/main.cpp src/check_image_data.cpp)

# Specify dependencies for the target
ament_target_dependencies(task_planner behaviortree_cpp_v3 rclcpp sensor_msgs)

# Install the executable
install(TARGETS task_planner
  DESTINATION lib/${PROJECT_NAME})

# Install XML files for Behavior Trees
install(DIRECTORY trees
  DESTINATION share/${PROJECT_NAME}/trees)

ament_package()
