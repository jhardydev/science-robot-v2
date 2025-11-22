# Duckiebot Science Fair Robot v2.0

A gesture-controlled robot that detects waving children, steers toward them, and performs dance routines when triggered by hand gestures. Built for Duckiebot platforms using ROS.

**Version 2.0 - Ubuntu 22.04 with Catkin Package Structure**

## What's New in v2.0

- ✅ **Ubuntu 22.04 LTS** support
- ✅ **Catkin package structure** for better ROS integration
- ✅ **ROS launch files** for easier deployment
- ✅ **Improved package management** with `package.xml` and `CMakeLists.txt`
- ✅ **Better ROS integration** following Duckietown best practices

## Features

- **Wave Detection**: Uses computer vision to detect waving children and steer the robot toward them
- **Gesture Recognition**: Recognizes hand gestures using MediaPipe
- **Dance Routines**: Performs predefined dance sequences when a special gesture is detected
- **Treat Dispensing**: Architecture ready for future treat dispensing mechanism
- **NVIDIA GPU Acceleration**: CUDA and VPI acceleration for improved performance (optional)
- **ROS Integration**: Uses ROS topics for motor control and camera access
- **Docker Support**: Containerized for easy deployment on Duckiebot platforms
- **Catkin Package**: Proper ROS package structure for better integration

## Hardware Requirements

- Duckiebot with ROS running
- Camera (access via ROS camera node)
- Motor control via Duckietown's wheels_driver_node

## Software Requirements

- **Ubuntu 22.04 LTS** (upgraded from 20.04)
- **ROS Noetic** (ROS 1)
- Python 3.10+
- Docker (for containerized deployment)
- ARM64 architecture (tested on Jetson Nano)

## Quick Start with Docker (Recommended)

```bash
# Build and run with helper script
./docker-run.sh --build

# Or use docker-compose
docker-compose up --build

# Or use ROS launch file directly
roslaunch science_robot science_robot.launch robot_name:=robot1
```

## Project Structure (v2.0 - Catkin Package)

```
science-robot-v2/
├── packages/
│   └── science_robot/          # Catkin package
│       ├── package.xml          # ROS package definition
│       ├── CMakeLists.txt       # Catkin build configuration
│       ├── setup.py             # Python package setup
│       ├── src/
│       │   ├── science_robot/   # Python package
│       │   │   ├── __init__.py
│       │   │   ├── config.py
│       │   │   ├── camera.py
│       │   │   ├── motor_controller.py
│       │   │   └── ...          # Other modules
│       │   └── science_robot_node.py  # Main ROS node
│       ├── launch/
│       │   └── science_robot.launch   # ROS launch file
│       └── scripts/
│           └── science_robot_node    # Executable wrapper
├── Dockerfile                  # Ubuntu 22.04 compatible
├── docker-compose.yml
├── docker-entrypoint.sh
├── docker-run.sh
├── requirements.txt
└── README.md
```

## Installation

### Using Docker (Recommended)

1. **Build the container**:
   ```bash
   docker build -t science-robot-v2:latest .
   ```

2. **Run with docker-compose**:
   ```bash
   docker-compose up
   ```

3. **Or run with helper script**:
   ```bash
   ./docker-run.sh --build
   ```

### Manual Installation (Without Docker)

1. **Install ROS dependencies**:
   ```bash
   sudo apt-get install ros-noetic-rospy \
       ros-noetic-duckietown-msgs \
       ros-noetic-cv-bridge \
       ros-noetic-sensor-msgs \
       ros-noetic-std-msgs \
       ros-noetic-catkin \
       python3-catkin-tools \
       python3-rospkg
   ```

2. **Build the Catkin workspace**:
   ```bash
   cd packages
   catkin_make
   source devel/setup.bash
   ```

3. **Install Python dependencies**:
   ```bash
   pip3 install -r requirements.txt
   ```

4. **Run the node**:
   ```bash
   roslaunch science_robot science_robot.launch robot_name:=robot1
   ```

## Configuration

Edit `packages/science_robot/src/science_robot/config.py` to adjust:

- **ROS settings**: Robot name and topic names (auto-configured)
- **Camera settings**: Resolution, FPS (source is ROS topic)
- **Motor control**: Speeds and thresholds (control via ROS topics)
- **Wave detection**: Sensitivity, motion thresholds
- **Navigation**: Steering gain, dead zones
- **Gestures**: Confidence thresholds, hold times
- **NVIDIA acceleration**: CUDA, VPI, and GPU preprocessing options (optional)

## Usage

### Running as ROS Node

```bash
# Using launch file (recommended)
roslaunch science_robot science_robot.launch robot_name:=robot1

# Or directly
rosrun science_robot science_robot_node
```

### Running in Docker

```bash
# Build and run
./docker-run.sh --build

# With custom robot name
./docker-run.sh --build --robot-name duckiebot

# With display output
./docker-run.sh --build --display-output
```

### Controls

- **'q'**: Quit the program
- **'s'**: Emergency stop (stops motors immediately)

### Gestures

- **Wave**: Side-to-side hand motion to attract robot's attention
- **Dance Trigger**: Open hand (all fingers extended) held for 1 second
- **Treat Trigger** (future): Thumb and pinky extended (shaka gesture) held for 2 seconds

## Migration from v1.0

If you're upgrading from the original version:

1. **Directory structure changed**: Code is now in `packages/science_robot/src/science_robot/`
2. **Imports changed**: Use `from science_robot import config` instead of `import config`
3. **Launch files**: Use `roslaunch science_robot science_robot.launch` instead of running `main.py` directly
4. **Dockerfile**: Updated for Ubuntu 22.04 and Catkin build system

## Troubleshooting

### Catkin workspace not building

```bash
# Ensure you have catkin tools installed
sudo apt-get install python3-catkin-tools

# Build manually
cd packages
catkin_make
source devel/setup.bash
```

### ROS package not found

```bash
# Ensure workspace is sourced
source packages/devel/setup.bash

# Verify package is found
rospack find science_robot
```

### Camera not detected

- Verify ROS camera node is running: `rostopic list | grep camera`
- Check camera topic: `rostopic hz /robot1/camera_node/image/compressed`
- Verify robot name in `config.py` matches your Duckiebot

### Motors not responding

- Verify ROS wheels_driver_node is running
- Check robot's FSM mode allows external commands
- Verify motor topic name matches your setup

## License

This project is created for educational purposes (science fair project).

## Credits

Built using:
- OpenCV for computer vision (with CUDA acceleration)
- MediaPipe for hand detection
- ROS (Robot Operating System) for hardware interface
- Duckietown ROS stack for robot integration
- NVIDIA VPI for GPU-accelerated image processing (optional)

