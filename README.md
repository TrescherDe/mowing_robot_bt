# Mowing Robot Behavior Tree (mowing_robot_bt)

This repository provides a ROS2-based task planner for an autonomous mowing robot using behavior trees.

## Setup Instructions

### 1. Clone this Repository

First, clone this repository into your ROS2 workspace:

```bash
git clone <this-repo-url> ~/ros_ws/src/mowing_robot_bt
```

Replace `<this-repo-url>` with the actual URL of this repository.

### 2. Clone and Setup Dependency Repository

Clone the required dependency repository:

```bash
git clone <dependency-repo-url> ~/ros_ws/src/dependency_repo
```

Follow the setup instructions provided in the dependency repository to ensure proper installation.

### 3. Source the Virtual Environment

Activate the virtual environment created during the dependency setup:

```bash
source <path-to-venv>/bin/activate
```

Replace `<path-to-venv>` with the correct path to the virtual environment directory.

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
<cmd1-to-start-your-camera>
```

Replace `<cmd1-to-start-your-camera>` with the actual command for starting your camera.

- **Option 2:** Send dummy thermal image messages to simulate the camera feed:

```bash
ros2 topic pub /thermal_image sensor_msgs/Image "<message-content>" -r 5
```

Replace `<message-content>` with the appropriate content for a `sensor_msgs/Image` message.

## Notes
- Ensure all dependencies are installed correctly.
- The task planner relies on camera input; you can simulate it with dummy messages if necessary.
- Adjust paths and URLs according to your setup.
