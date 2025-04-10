cmake_minimum_required(VERSION 3.8)
project(mowing_robot_bt)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Python3 COMPONENTS Development REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include  # Include the package's header files
  ${Python3_INCLUDE_DIRS}  # Include Python headers
)

# Add the executables
add_executable(task_planner src/main.cpp src/creature_detection.cpp)

# Specify dependencies for the target
ament_target_dependencies(task_planner 
  behaviortree_cpp_v3
  rclcpp
  sensor_msgs
  cv_bridge
  Python3
  tf2
  tf2_ros
  tf2_geometry_msgs
  OpenCV
  geometry_msgs
  vision_msgs)

# Install the executable
install(TARGETS task_planner
  DESTINATION lib/${PROJECT_NAME})

# Add Python NeuralNet script
install(PROGRAMS
  scripts/NeuralNet.py
  scripts/IPM.py
  scripts/IPM_calibration.py
  scripts/static_tf2_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install XML files for Behavior Trees
install(DIRECTORY trees
  DESTINATION share/${PROJECT_NAME}/trees)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install launthe model
install(DIRECTORY model/
  DESTINATION share/${PROJECT_NAME}/model
)

ament_package()
