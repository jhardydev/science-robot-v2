# Host System Installation Guide

This guide helps you install dependencies on the robot's host system (not in Docker).

## ARM64 Systems (Duckiebot)

On ARM64 systems, some packages like `opencv-python` don't have pre-built wheels and require building from source. It's easier to use the system packages instead.

### Step 1: Install System Dependencies

```bash
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-opencv \
    libopencv-dev \
    python3-dev \
    build-essential \
    cmake \
    libgl1-mesa-glx \
    libglib2.0-0
```

### Step 2: Install Python Dependencies (without OpenCV)

Since OpenCV is installed via apt, use the modified requirements file:

```bash
cd ~/science-robot-v2
pip3 install --user -r requirements-no-opencv.txt
```

**Or**, if you prefer to install dependencies manually:

```bash
pip3 install --user numpy>=1.24.0
pip3 install --user flask>=2.0.0
pip3 install --user psutil>=5.9.0

# MediaPipe (may fail on ARM64 - see below)
pip3 install --user "mediapipe>=0.10.8" || echo "MediaPipe install failed - may need to build from source"
```

### Step 3: Install ROS Dependencies

```bash
# For ROS Noetic (Ubuntu 22.04)
sudo apt-get install -y \
    ros-noetic-rospy \
    ros-noetic-duckietown-msgs \
    ros-noetic-cv-bridge \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs \
    ros-noetic-catkin \
    python3-catkin-tools \
    python3-rospkg
```

### Step 4: Verify OpenCV Installation

```bash
python3 -c "import cv2; print('OpenCV version:', cv2.__version__)"
```

You should see the OpenCV version printed. If you get an import error, the system package isn't installed correctly.

### MediaPipe on ARM64

MediaPipe may not install cleanly on ARM64. If the pip install fails:

1. **Option 1**: Continue without MediaPipe - the robot will use custom gesture detection
2. **Option 2**: Try installing build dependencies:
   ```bash
   sudo apt-get install -y \
       python3-dev \
       build-essential \
       cmake \
       protobuf-compiler \
       libprotobuf-dev
   pip3 install --user "mediapipe>=0.10.8"
   ```
3. **Option 3**: Build MediaPipe from source (complex, see MediaPipe documentation)

The robot will work without MediaPipe Gesture Recognizer, but will use the fallback custom detection instead.

## x86_64 Systems

If you're installing on an x86_64 system, you can use the standard requirements.txt:

```bash
pip3 install --user -r requirements.txt
```

## Troubleshooting

### "pip3: command not found"
```bash
sudo apt-get update
sudo apt-get install python3-pip
```

### "opencv-python build fails"
Use the system package instead:
```bash
sudo apt-get install python3-opencv libopencv-dev
pip3 install --user -r requirements-no-opencv.txt
```

### "Permission denied" errors
Use `--user` flag to install in user space:
```bash
pip3 install --user -r requirements-no-opencv.txt
```

### Verify Installation
```bash
python3 -c "import cv2, numpy, flask, psutil; print('All packages imported successfully!')"
```

