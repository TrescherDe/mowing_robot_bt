cmake_minimum_required(VERSION 3.8)
project(mowing_robot_bt)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)  # Add cv_bridge
find_package(Python3 COMPONENTS Development REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include  # Include the package's header files
  /workspaces/ros2_jazzy/01_repos/PMBW_Object_Detection_In_Thermal_Images # Include EfficientDet repo
  ${Python3_INCLUDE_DIRS}  # Include Python headers
)

# Add the executables
add_executable(task_planner src/main.cpp src/creature_detection.cpp src/complete_coverage_path.cpp src/update_ccp_map.cpp) #  src/check_if_true.cpp

# Specify dependencies for the target
ament_target_dependencies(task_planner behaviortree_cpp_v3 rclcpp sensor_msgs cv_bridge Python3)

# Install the executable
install(TARGETS task_planner
  DESTINATION lib/${PROJECT_NAME})

# Install XML files for Behavior Trees
install(DIRECTORY trees
  DESTINATION share/${PROJECT_NAME}/trees)

ament_package()
