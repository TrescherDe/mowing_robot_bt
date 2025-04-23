# Mowing Robot Behavior Tree (mowing_robot_bt)

This repository provides a ROS2-based creature detection node used in a behavior tree for an autonomous mowing robot.  
The dataset used for training the model to detect hedgehogs can be found here: https://github.com/TrescherDe/hedgehog_dataset

## Setup Instructions

### 1. Use the Docker Image

Use the prebuilt Docker image to run the system inside a VS Code **Devcontainer**:  
https://hub.docker.com/repository/docker/trescherde/mowing_robot_tutorial/tags

<details>
<summary><strong> Setup Docker + Devcontainer (Raspberry Pi)</strong></summary>

#### Install Docker on Raspberry Pi 5 (Ubuntu 24.04)

1. Download Docker binaries for AArch64:  
   https://download.docker.com/linux/static/stable/

2. Extract the archive:
   ```bash
   tar xzvf docker-<version>.tgz
   ```

3. Move Docker binaries to system path:
   ```bash
   sudo cp docker/* /usr/bin/
   ```

4. Create Docker group and add your user:
   ```bash
   sudo groupadd docker
   sudo usermod -aG docker $USER
   ```

5. If permission is denied using Docker:
   ```bash
   ls -l /var/run/docker.sock
   sudo chown root:docker /var/run/docker.sock
   ```

More details: https://docs.docker.com/engine/install/binaries/#install-daemon-and-client-binaries-on-linux

---

#### Set up the Devcontainer (VS Code)

1. Install VS Code and the **Dev Containers** extension.

2. Create a folder:
   ```bash
   mkdir ros2_jazzy && cd ros2_jazzy
   mkdir .devcontainer && cd .devcontainer
   ```

3. **Create `Dockerfile`** with the following content:
   ```Dockerfile
   FROM trescherde/mowing_robot_tutorial:ros-jazzy-creature-detection-hailort-v1.0

   # Set ROS Domain ID
   ENV ROS_DOMAIN_ID=0
   ```

4. **Create `devcontainer.json`** in the same folder:
   ```json
   {
     "name": "ros2_jazzy",
     "build": {
       "dockerfile": "Dockerfile"
     },
     "runArgs": [
       "-it",
       "--net=host",
       "--pid=host",
       "--env=DISPLAY=${localEnv:DISPLAY}",
       "--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw",
       "--privileged",
       "--volume", "/dev:/dev",
       "--device=/dev/hailo0"
     ],
     "customizations": {
       "vscode": {
         "settings": {
           "terminal.integrated.shell.linux": "/bin/bash"
         },
         "mounts": [
           "type=bind,source=/lib/modules,target=/lib/modules",
           "type=bind,source=/usr/src,target=/usr/src"
         ],
         "remoteUser": "user",
         "extensions": [
           "ms-vscode.cmake-tools",
           "ms-python.python",
           "mhutchie.git-graph",
           "ms-vscode.makefile-tools",
           "ms-vscode.cpptools",
           "ms-azuretools.vscode-docker",
           "ms-vscode.cpptools",
           "ms-vscode.cpptools-extension-pack",
           "theumletteam.umlet"
         ]
       }
     },
	"postCreateCommand": "/bin/bash -c \"source /opt/creature_detection/bin/activate && pip install rosdep && pip install 'numpy<2' && sed -i 's|/workspaces/eduartrobotik_ros2_jazzy|/workspaces/ros2_jazzy|' ~/.bashrc\""
   }
   ```
   5. Navigate into your main folder (e.g. `ros2_jazzy`) in the terminal:
   ```bash
   cd ~/ros2_jazzy
   code .
   ```

   VS Code will open and show a popup: **"Reopen in container"** – click it to launch your devcontainer.

</details>

### 2. Setup HailoRT

#### a) Install the PCIe Driver (on the Raspberry Pi host system, outside of Docker)

   <details>
   <summary><strong>Setup PCIe Driver</strong></summary>
   
   1. Install dependencies:
      ```bash
      sudo apt update
      sudo apt install build-essential dkms
      ```
   
   2. Install kernel headers:
      ```bash
      apt search linux-headers-$(uname -r)
      sudo apt install linux-headers-<your-kernel-version>
      ```
      Example for Raspberry Pi:
      ```bash
      sudo apt install linux-headers-6.8.0-1020-raspi
      ```
   
   3. Reboot the Raspberry Pi:
      ```bash
      sudo reboot
      ```
   
   4. Download and install the Hailo PCIe driver:
      > You must create a developer account at [hailo.ai](https://hailo.ai/developer-zone/software-downloads/) and download the file:
      > `hailort-pcie-driver_4.20.0_all.deb`
   
      Then install it:
      ```bash
      sudo dpkg -i hailort-pcie-driver_4.20.0_all.deb
      ```
   
   5. Reboot again:
      ```bash
      sudo reboot
      ```
   
   ---
   </details>

#### b) HailoRT & Python (inside the Docker container)

   <details>
   <summary><strong>Setup HailoRT</strong></summary>
       
   1. Install the Hailo runtime `.deb`:
      > Download `hailort_4.20.0_arm64.deb` from [Hailo Developer Zone](https://hailo.ai/developer-zone/software-downloads/)
   
      Then inside the container:
      ```bash
      sudo dpkg -i hailort_4.20.0_arm64.deb
      ```
   
   2. Install the Hailo Python bindings:
   
      There is currently **no official Python 3.12 wheel** available.  
      You must build the Python bindings from source.
      
      Follow the build instructions here:  
      https://github.com/TrescherDe/ros2-hailort
   
   ---
 </details>

### 3. Clone this Repository

Then, create a ROS 2 workspace and clone this repository into it:

```bash
mkdir -p ros_ws/src && cd ros_ws/src
git clone https://github.com/TrescherDe/mowing_robot_bt.git
```

### 4. Clone and Set Up the Inverse Perspective Mapping Repository

Clone the Inverse Perspective Mapping (IPM) repository **also into the ROS workspace**:

Installation instructions can be found here: 

[install ipm library → Installation — Inverse Perspective Mapping  documentation](https://ipm-docs.readthedocs.io/en/latest/installation.html#installation)

### 5. Clone and Setup the Optris Repository

Clone the required dependency repository  **also into the ROS workspace**:

```bash
git clone https://github.com/evocortex/optris_drivers2/tree/master
```

Follow the setup instructions provided in the dependency repository.

### 6. Source the Virtual Environment

Activate the virtual environment included in the Docker image:

```bash
source /opt/creature_detection/bin/activate
```

### 7. Build and Source the Workspace

Navigate to your ROS workspace and build the packages:

```bash
cd ~/ros_ws
colcon build
source install/setup.bash
```

### 8. Run the Creature Detection

Launch the Creature Detection for the mowing robot:

```bash
ros2 launch mowing_robot_bt CreatureDetection.py
```

### 9. Start Your Camera or Send Dummy Messages

- **Option 1:** Start your camera using the appropriate command:

   ```bash
   ros2 run optris_drivers2 optris_imager_node
   ```

- **Option 2:** Send dummy thermal image messages to simulate the camera feed:

   ```bash
   ros2 topic pub /thermal_image sensor_msgs/Image -r 5
   ```

## Example Integration on a Robot Which Uses Nav2

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
   **Extend the Costmap Configuration**  
     In both the local and global costmap sections, locate and update the `observation_sources` entry.  
     The default typically looks like this:
   
     ```yaml
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
   
     Update it to include the `detected_creatures` plugin:
   
     ```yaml
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

     
6. **Start the Nav2 docker on the Robot**
    ```bash
     cd edu_docker/nav2/
     docker compose up -d
     ```

## Training of a custom detection model

<details>
<summary><strong>Training of a yolov8n model</strong></summary>

1. Install ultralytics: https://github.com/ultralytics/ultralytics → `pip install ultralytics`

2. Prepare the training data  
    1. Setup a dataset folder containing the structure: `train/`, `val/`, `test/`  
    2. Add a `yaml` in your dataset folder containing the paths to the train, val, and test dataset and the number of classes used, etc. For example:
        ```
        train: rgb_yolo/train
        val: rgb_yolo/val
        test: rgb_yolo/test
        nc: 1
        names: [hedgehog]
        ```
        You can use the hedgehog dataset used in the thesis by cloning the [hedgehog dataset repo](https://github.com/TrescherDe/hedgehog_dataset) and utilizing the [rgb_yolo](https://github.com/TrescherDe/hedgehog_dataset/tree/master/rgb_yolo) or [thermal_8bit_yolo](https://github.com/TrescherDe/hedgehog_dataset/tree/master/thermal_8bit_yolo) depending if you want to use RGB data or thermal data.

3. Follow the Ultralytics training guide: https://docs.ultralytics.com/modes/train/  
    Train a YOLO model using the terminal:  
    ```
    yolo task=detect mode=train data=<path_to_dataset.yaml> model=yolov8n.pt epochs=50 imgsz=640 patience=10 project=<path_to_logs> name=run
    ```

4. Ultralytics will provide a folder after training containing the weights named `best.pt` which you can use to detect on the data.

5. Use this weight to test your model on your val / test data:  
    ```
    yolo task=detect mode=val model=<path_to_best.pt> data=<path_to_dataset.yaml>
    ```
    Note: the model weights should by default be in the logs folder → `logs/.../best.pt`

Now you have a working object detection model for your data. Next, we will transform it to a format suitable for use on the AI Kit.

1. To use the model on the AI Kit you first have to convert the model to ONNX:  
    ```
    yolo export model=<path_to_best.pt> format=onnx imgsz=640
    ```

2. Use the Hailo AI Software Suite Docker which you can download in the [Hailo Developer Zone](https://hailo.ai/developer-zone/) under “Software Downloads”:
    ```
    sudo docker run -it --rm \
    --gpus all \
    -v <path_to_calibration_data>:/workspace/calibration_data \
    -v <path_to_weights>:/workspace/weights \
    hailo_ai_sw_suite_2025-01:1
    ```

3. Use the Hailo Model Zoo command-line tool to convert the ONNX to the Hailo Executable Format (HEF format):
    ```
    hailomz compile yolov8n \
    --ckpt /workspace/weights/<model.onnx> \
    --hw-arch hailo8l \
    --calib-path /workspace/calibration_data \
    --classes 1 \
    --performance
    ```

4. You're done. Adapt the path in the [NeuralNet node](https://github.com/TrescherDe/mowing_robot_bt/blob/main/scripts/NeuralNet.py) to your model weights if needed.
</details>

## Notes
- Ensure all dependencies are installed correctly.
- The task planner relies on camera input; you can simulate it with dummy messages if necessary.
- If you train a custom model with different class names, more classes, other image sizes, etc., you have to update the code to fit your configuration in the [NeuralNet node](https://github.com/TrescherDe/mowing_robot_bt/blob/main/scripts/NeuralNet.py)
