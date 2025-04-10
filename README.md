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

## Notes
- Ensure all dependencies are installed correctly.
- The task planner relies on camera input; you can simulate it with dummy messages if necessary.
- Adjust paths and URLs according to your setup.
