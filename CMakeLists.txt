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
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include  # Include the package's header files
  /workspaces/eduartrobotik_ros2_jazzy/01_repos/PMBW_Object_Detection_In_Thermal_Images # Include EfficientDet repo
  ${Python3_INCLUDE_DIRS}  # Include Python headers
)

# Add the executables
add_executable(task_planner src/main.cpp src/creature_detection.cpp src/complete_coverage_path.cpp src/add_obstacle.cpp src/check_collision_free_ccp_path_available.cpp)

# Specify dependencies for the target
ament_target_dependencies(task_planner behaviortree_cpp_v3 rclcpp sensor_msgs cv_bridge Python3 std_srvs tf2 tf2_ros tf2_geometry_msgs OpenCV geometry_msgs vision_msgs)

# Install the executable
install(TARGETS task_planner
  DESTINATION lib/${PROJECT_NAME})

# Install the launch directory
#install(DIRECTORY launch/
#  DESTINATION share/${PROJECT_NAME}/launch)

# Install XML files for Behavior Trees
install(DIRECTORY trees
  DESTINATION share/${PROJECT_NAME}/trees)

ament_package()
