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
    python3.10-dev \
    libopencv-dev \
    python3-opencv \
    libgl1-mesa-glx \
    libglib2.0-0 \
    xvfb \
    x11vnc \
    x11-utils \
    fontconfig-config \
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

# Install mediapipe
RUN pip3 install --no-cache-dir --default-timeout=900 --retries=10 \
    mediapipe>=0.10.0 || \
    (echo "MediaPipe installation failed, retrying..." && \
     pip3 install --no-cache-dir --default-timeout=1200 --retries=15 \
     mediapipe>=0.10.0)

# Copy application code
COPY . .

# Build Catkin workspace
RUN if [ -d packages ]; then \
        cd /code && \
        source /opt/ros/noetic/setup.bash && \
        catkin_make -C packages || catkin build -w packages || true; \
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

