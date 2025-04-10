# Mowing Robot Behavior Tree (mowing_robot_bt)

This repository provides a ROS2-based creature detection node used in a behavior tree for an autonomous mowing robot.

## Setup Instructions

### 1. Use the Docker Image

Use the prebuilt Docker image as a development environment:  
https://hub.docker.com/repository/docker/trescherde/mowing_robot_tutorial/tags

### 2. Clone this Repository

First, clone this repository into your ROS2 workspace:

```bash
git clone https://github.com/TrescherDe/mowing_robot_bt.git
```

### 3. Clone and Setup Inverse perspective mapping Repository

Installation instructions can be found here: 

[install ipm library → Installation — Inverse Perspective Mapping  documentation](https://ipm-docs.readthedocs.io/en/latest/installation.html#installation)

### 4. Clone and Setup the Optris Repository

Clone the required dependency repository:

```bash
git clone https://github.com/evocortex/optris_drivers2/tree/master
```

Follow the setup instructions provided in the dependency repository.

### 5. Source the Virtual Environment

Activate the virtual environment included in the Docker image:

```bash
source /opt/creature_detection/bin/activate
```

### 6. Build and Source the Workspace

Navigate to your ROS2 workspace and build the packages:

```bash
cd ~/ros_ws
colcon build
source install/setup.bash
```

### 7. Run the Creature Detection

Launch the Creature Detection for the mowing robot:

```bash
ros2 launch mowing_robot_bt CreatureDetection.py
```

### 8. Start Your Camera or Send Dummy Messages

- **Option 1:** Start your camera using the appropriate command:

```bash
ros2 run optris_drivers2 optris_imager_node
```

- **Option 2:** Send dummy thermal image messages to simulate the camera feed:

```bash
ros2 topic pub /thermal_image sensor_msgs/Image -r 5
```

## Integration with the Eduard Robot

The system is modular and can be tested, for example, on the **Eduard robot from EduArt Robotik GmbH**.

### Steps to Integrate

1. **Connect to the Robot**  
   Connect your device (e.g., Raspberry Pi or laptop) to the robot's Wi-Fi network.

2. **Check Available Topics**  
   Use the following command to verify communication:
   ```bash
   ros2 topic list
   ```
3. **Verify All Necessary Docker Containers Are Running**  
   Ensure that the required Docker containers are active on the robot.

   - Connect to the robot via SSH:
     ```bash
     ssh user@<robot-ip>
     ```

   - Check the currently running containers:
     ```bash
     docker ps
     ```

   - If the **sensor container** is not running:
     ```bash
     cd edu_docker/sensor/rplidar/
     docker compose up -d
     ```

   - If the **SLAM Toolbox** is not running:
     ```bash
     cd edu_docker/slam_tool_box/
     docker compose up -d
     ```

5. **Edit the Nav2 Configuration**  
   On the Eduard robot, open the following file:
   ```bash
   nav2/launch_content/navigation.yaml
   ```
4. **Extend the Costmap Configuration**  
  In both the local and global costmap sections, locate and update the observation_sources entry.
  The default typically looks like this:
  ```bash
  observation_sources: scan
  scan:
    topic: scan
    max_obstacle_height: 2.0
    clearing: True
    marking: True
    data_type: "LaserScan"
    raytrace_max_range: 3.0
    raytrace_min_range: 0.0
    obstacle_max_range: 2.5
    obstacle_min_range: 0.0
  ```
  Update it to include the detected_creatures:
  ```bash
  observation_sources: scan detected_creatures
  scan:
    topic: scan
    max_obstacle_height: 2.0
    clearing: True
    marking: True
    data_type: "LaserScan"
    raytrace_max_range: 3.0
    raytrace_min_range: 0.0
    obstacle_max_range: 2.5
    obstacle_min_range: 0.0
  detected_creatures:
    obstacle_topic: "/eduard/fred/detected_creatures"
    max_obstacle_height: 0.5
    clearing: False
    marking: True
    data_type: "PointCloud2"
    obstacle_max_range: 2.0
    obstacle_min_range: 0.2
  ```
5. **Start the Nav2 docker on the Robot**
    ```bash
     cd edu_docker/nav2/
     docker compose up -d
     ```
    
## Notes
- Ensure all dependencies are installed correctly.
- The task planner relies on camera input; you can simulate it with dummy messages if necessary.
