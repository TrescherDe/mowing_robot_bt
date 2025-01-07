# Mowing Robot Behavior Tree (mowing_robot_bt)

This repository provides a ROS2-based task planner for an autonomous mowing robot using behavior trees.

## Setup Instructions

### 1. Clone this Repository

First, clone this repository into your ROS2 workspace

### 2. Clone and Setup Dependency Repository

Clone the required dependency repository:

```bash
git clone https://github.com/TrescherDe/PMBW_Object_Detection_In_Thermal_Images
```

Follow the setup instructions provided in the dependency repository.

### 3. Clone and Setup the Optris Repository

Clone the required dependency repository:

```bash
git clone https://github.com/evocortex/optris_drivers2/tree/master
```

Follow the setup instructions provided in the dependency repository.

### 4. Source the Virtual Environment

Activate the virtual environment created during the dependency setup:

```bash
source <path-to-venv>/bin/activate
```

### 4. Build the Workspace

Navigate to your ROS2 workspace and build the packages:

```bash
cd ~/ros_ws
colcon build
```

### 5. Run the Task Planner

Launch the task planner node for the mowing robot:

```bash
ros2 run mowing_robot_bt task_planner
```

### 6. Start Your Camera or Send Dummy Messages

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
