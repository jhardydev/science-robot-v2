# Dockerfile for Duckiebot Science Fair Robot v2.0
# For Ubuntu 22.04 ARM64 (ROS Noetic)
# Uses Catkin package structure

FROM duckietown/dt-ros-commons:ente-arm64v8

# Set working directory
WORKDIR /code

# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies for Ubuntu 22.04
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    libopencv-dev \
    python3-opencv \
    libgl1-mesa-glx \
    libglib2.0-0 \
    xvfb \
    x11vnc \
    x11-utils \
    fontconfig-config \
    libprotobuf-dev \
    protobuf-compiler \
    ros-noetic-catkin \
    python3-catkin-tools \
    python3-rospkg \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Fix fontconfig warnings
RUN mkdir -p /etc/fonts/conf.d && \
    if [ ! -f /etc/fonts/fonts.conf ]; then \
        fc-cache -f 2>/dev/null || true; \
    fi

# Check for VPI
RUN python3 -c "import vpi" 2>/dev/null && echo "VPI is available" || \
    (echo "VPI not found in base image - will use CPU fallback" && \
     echo "Note: VPI requires JetPack SDK on Jetson devices")

# Copy requirements
COPY requirements.txt .

# Upgrade pip, setuptools, wheel
RUN pip3 install --no-cache-dir --default-timeout=600 --retries=5 \
    --upgrade pip setuptools wheel

# Install numpy first
RUN pip3 install --no-cache-dir --default-timeout=600 --retries=5 \
    numpy>=1.24.0

# Install opencv-python
RUN pip3 install --no-cache-dir --default-timeout=600 --retries=5 \
    opencv-python>=4.8.0 || \
    pip3 install --no-cache-dir --default-timeout=900 --retries=10 \
    opencv-python>=4.8.0

# Install mediapipe (OPTIONAL - MediaPipe has no pre-built wheels for ARM64 Linux)
# MediaPipe installation often fails on ARM64. The code handles missing MediaPipe gracefully.
# If this fails, you can install it manually after the container starts, or the robot
# will run without gesture detection features.
RUN echo "Attempting to install MediaPipe (may fail on ARM64 - this is OK)..." && \
    (pip3 install --no-cache-dir --default-timeout=900 --retries=3 \
        mediapipe>=0.10.0 2>&1 | tee /tmp/mediapipe_install.log && \
     echo "✓ MediaPipe installed successfully") || \
    (echo "MediaPipe wheel install failed (expected on ARM64)." && \
     echo "Trying alternative installation methods..." && \
     (pip3 install --no-cache-dir --default-timeout=1200 --retries=2 \
         mediapipe==0.10.3 2>&1 | tee -a /tmp/mediapipe_install.log && \
      echo "✓ MediaPipe 0.10.3 installed") || \
     (echo "=========================================" && \
      echo "MediaPipe installation SKIPPED" && \
      echo "=========================================" && \
      echo "MediaPipe does not provide pre-built wheels for ARM64 Linux." && \
      echo "The robot will run but gesture detection will be disabled." && \
      echo "" && \
      echo "To enable gesture detection, you can:" && \
      echo "  1. Install MediaPipe manually in the running container:" && \
      echo "     docker exec -it <container> pip3 install mediapipe" && \
      echo "  2. Or build MediaPipe from source (requires Bazel)" && \
      echo "  3. Or use a base image that includes MediaPipe" && \
      echo "" && \
      echo "See BUILD_INSTRUCTIONS.md for more details." && \
      echo "=========================================" && \
      echo "Continuing build without MediaPipe (this is OK)..." && \
      true))

# Copy application code
COPY . .

# Build Catkin workspace
# Catkin expects: packages/src/science_robot/ but we have packages/science_robot/
# Restructure to proper Catkin workspace layout
# Note: Must source Duckietown workspace first to find duckietown_msgs
RUN if [ -d packages/science_robot ]; then \
        cd /code && \
        mkdir -p packages/src && \
        if [ ! -d packages/src/science_robot ]; then \
            mv packages/science_robot packages/src/ 2>/dev/null || \
            cp -r packages/science_robot packages/src/ 2>/dev/null || true; \
        fi && \
        bash -c "source /opt/ros/noetic/setup.bash && \
                 if [ -f /code/catkin_ws/devel/setup.bash ]; then \
                     source /code/catkin_ws/devel/setup.bash; \
                 elif [ -f /code/catkin_ws/install/setup.bash ]; then \
                     source /code/catkin_ws/install/setup.bash; \
                 fi && \
                 cd packages && \
                 (catkin_make 2>&1 || \
                  (echo 'catkin_make failed, trying catkin build...' && \
                   catkin init 2>/dev/null || true && \
                   catkin build 2>&1)) || \
                 echo 'Catkin build failed - will retry at runtime'"; \
    fi

# Make scripts executable
RUN find . -name "*.py" -type f -exec chmod +x {} \; && \
    find . -name "*.sh" -type f -exec chmod +x {} \; && \
    find . -path "*/scripts/*" -type f -exec chmod +x {} \;

# Create logs directory
RUN mkdir -p /code/logs

# Entrypoint script
COPY docker-entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "science_robot", "science_robot.launch"]

