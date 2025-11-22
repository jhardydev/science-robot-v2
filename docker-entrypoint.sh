#!/bin/bash
# Entrypoint script for Duckiebot Science Fair Robot v2.0 container
# Ubuntu 22.04 with Catkin package structure

# Don't exit on error - let the Python script handle errors
# set -e

# Skip sourcing /environment.sh - the Duckietown base image entrypoint already does this
echo "Duckietown environment should already be sourced by base image entrypoint"

# Source ROS Noetic setup (Ubuntu 22.04 uses Noetic)
if [ -f /opt/ros/noetic/setup.bash ]; then
    echo "Sourcing ROS Noetic setup..."
    source /opt/ros/noetic/setup.bash || echo "Warning: Failed to source ROS Noetic"
else
    echo "Warning: /opt/ros/noetic/setup.bash not found"
fi

# Source Duckietown workspace to make duckietown_msgs available
# The base image has the workspace at /code/catkin_ws
if [ -f /code/catkin_ws/devel/setup.bash ]; then
    echo "Sourcing Duckietown workspace..."
    source /code/catkin_ws/devel/setup.bash || echo "Warning: Failed to source workspace"
elif [ -f /code/catkin_ws/install/setup.bash ]; then
    echo "Sourcing Duckietown workspace (install space)..."
    source /code/catkin_ws/install/setup.bash || echo "Warning: Failed to source workspace"
else
    echo "Warning: Duckietown workspace setup.bash not found"
    echo "Attempting to add workspace to PYTHONPATH manually..."
    if [ -d /code/catkin_ws/devel/lib/python3/dist-packages ]; then
        export PYTHONPATH="/code/catkin_ws/devel/lib/python3/dist-packages:$PYTHONPATH"
        echo "Added /code/catkin_ws/devel/lib/python3/dist-packages to PYTHONPATH"
    fi
fi

# Source Catkin workspace if it exists (v2.0 with Catkin package structure)
if [ -d /code/packages ]; then
    echo "Setting up Catkin workspace for science_robot package..."
    source /opt/ros/noetic/setup.bash
    
    # Try to source the workspace
    if [ -f /code/packages/devel/setup.bash ]; then
        source /code/packages/devel/setup.bash
        echo "✓ Sourced Catkin workspace (devel)"
    elif [ -f /code/packages/install/setup.bash ]; then
        source /code/packages/install/setup.bash
        echo "✓ Sourced Catkin workspace (install)"
    else
        echo "⚠ Catkin workspace not built, building now..."
        cd /code/packages
        # Clean up any conflicting build spaces
        if [ -d build ] && [ -f build/.catkin_tools ]; then
            echo "Removing old catkin build space..."
            rm -rf build devel install
        elif [ -d build ] && [ ! -f build/.catkin_tools ]; then
            echo "Removing old catkin_make build space..."
            rm -rf build devel
        fi
        # Try building with catkin_make first
        bash -c "source /opt/ros/noetic/setup.bash && \
                 catkin_make 2>&1" || \
        (echo "catkin_make failed, trying catkin build..." && \
         bash -c "source /opt/ros/noetic/setup.bash && \
                  catkin init && catkin build 2>&1") || \
        echo "Build failed, continuing..."
        if [ -f /code/packages/devel/setup.bash ]; then
            source /code/packages/devel/setup.bash
            echo "✓ Built and sourced Catkin workspace"
        elif [ -f /code/packages/install/setup.bash ]; then
            source /code/packages/install/setup.bash
            echo "✓ Built and sourced Catkin workspace (install space)"
        else
            echo "⚠ Could not build Catkin workspace, package may not be available"
        fi
    fi
fi

# Ensure ROS master is set (default to localhost if not provided)
if [ -z "$ROS_MASTER_URI" ]; then
    export ROS_MASTER_URI=http://localhost:11311
    echo "Using default ROS_MASTER_URI: $ROS_MASTER_URI"
else
    echo "Using ROS_MASTER_URI: $ROS_MASTER_URI"
fi

# Set ROS hostname if not set
if [ -z "$ROS_HOSTNAME" ]; then
    export ROS_HOSTNAME=$(hostname)
fi
echo "ROS_HOSTNAME: $ROS_HOSTNAME"

# Wait for ROS master to be available
echo "Waiting for ROS master at $ROS_MASTER_URI..."
timeout=30
elapsed=0
ROS_AVAILABLE=false

while [ $elapsed -lt $timeout ]; do
    if rostopic list > /dev/null 2>&1; then
        echo "ROS master is available!"
        ROS_AVAILABLE=true
        rostopic list | head -5
        break
    fi
    echo "  Waiting for ROS master... (${elapsed}/${timeout}s)"
    sleep 1
    elapsed=$((elapsed + 1))
done

if [ "$ROS_AVAILABLE" = false ]; then
    echo "WARNING: ROS master not available after ${timeout}s, proceeding anyway..."
    echo "The application may fail if ROS topics are not accessible."
fi

# Virtual display setup (if enabled)
USE_VIRTUAL_DISPLAY=${USE_VIRTUAL_DISPLAY:-false}
VIRTUAL_DISPLAY_NUM=${VIRTUAL_DISPLAY_NUM:-99}
VIRTUAL_DISPLAY_SIZE=${VIRTUAL_DISPLAY_SIZE:-1024x768x24}

if [ "$USE_VIRTUAL_DISPLAY" = "true" ]; then
    echo "Starting virtual display (Xvfb) on :${VIRTUAL_DISPLAY_NUM}..."
    Xvfb :${VIRTUAL_DISPLAY_NUM} -screen 0 ${VIRTUAL_DISPLAY_SIZE} > /dev/null 2>&1 &
    XVFB_PID=$!
    sleep 1
    
    if kill -0 $XVFB_PID 2>/dev/null; then
        export DISPLAY=:${VIRTUAL_DISPLAY_NUM}
        export DISPLAY_OUTPUT=true
        echo "Virtual display started successfully on ${DISPLAY}"
        
        ENABLE_VNC=${ENABLE_VNC:-false}
        VNC_PORT=${VNC_PORT:-5900}
        if [ "$ENABLE_VNC" = "true" ]; then
            echo "Starting VNC server on port ${VNC_PORT}..."
            x11vnc -display :${VIRTUAL_DISPLAY_NUM} -nopw -forever -shared -rfbport ${VNC_PORT} > /dev/null 2>&1 &
            VNC_PID=$!
            sleep 1
            if kill -0 $VNC_PID 2>/dev/null; then
                echo "VNC server started on port ${VNC_PORT}"
            else
                echo "Warning: Failed to start VNC server"
            fi
        fi
    else
        echo "Warning: Failed to start virtual display, falling back to headless mode"
        export DISPLAY_OUTPUT=false
        export QT_QPA_PLATFORM=offscreen
    fi
elif [ -z "$DISPLAY" ]; then
    if [ "$DISPLAY_OUTPUT" = "true" ]; then
        echo "Warning: DISPLAY_OUTPUT=true but DISPLAY is not set"
        echo "  Falling back to headless mode"
        export DISPLAY_OUTPUT=false
        export QT_QPA_PLATFORM=offscreen
    else
        echo "No display available, running in headless mode"
        export DISPLAY_OUTPUT=${DISPLAY_OUTPUT:-false}
        export QT_QPA_PLATFORM=offscreen
    fi
else
    echo "Using X11 display: ${DISPLAY}"
    if command -v xdpyinfo > /dev/null 2>&1; then
        echo "Checking X11 connection..."
        if timeout 2 xdpyinfo > /dev/null 2>&1; then
            echo "✓ X11 connection is ready"
        else
            echo "⚠ X11 connection check timed out (may still work)"
            sleep 1
        fi
    fi
    
    if [ "$DISPLAY_OUTPUT" != "false" ]; then
        export DISPLAY_OUTPUT=true
    else
        export DISPLAY_OUTPUT=false
        export QT_QPA_PLATFORM=offscreen
    fi
fi

# Set Qt backend based on display availability
if [ "$DISPLAY_OUTPUT" = "false" ]; then
    export QT_QPA_PLATFORM=offscreen
else
    unset QT_QPA_PLATFORM
fi

# Setup VPI library paths (if VPI is mounted from host)
VPI_FOUND=false

if [ -n "$VPI_PARENT_DIR" ]; then
    HOST_VPI_PARENT="/host$VPI_PARENT_DIR"
    if [ -d "$HOST_VPI_PARENT" ]; then
        export PYTHONPATH="$HOST_VPI_PARENT:$PYTHONPATH"
        echo "Added $HOST_VPI_PARENT to PYTHONPATH for VPI"
        VPI_FOUND=true
    fi
fi

if [ "$VPI_FOUND" = false ]; then
    VPI_PATHS=(
        "/host/usr/lib/python3/dist-packages/vpi"
        "/host/usr/local/lib/python3/dist-packages/vpi"
        "/usr/lib/python3/dist-packages/vpi"
    )
    
    for VPI_PATH in "${VPI_PATHS[@]}"; do
        if [ -d "$VPI_PATH" ] || [ -f "${VPI_PATH}.py" ]; then
            VPI_PARENT=$(dirname "$VPI_PATH" 2>/dev/null || echo "")
            if [[ "$VPI_PARENT" == /host/* ]]; then
                HOST_PARENT="$VPI_PARENT"
            else
                HOST_PARENT="/host$VPI_PARENT"
            fi
            
            if [ -n "$HOST_PARENT" ] && [ -d "$HOST_PARENT" ]; then
                export PYTHONPATH="$HOST_PARENT:$PYTHONPATH"
                echo "Added $HOST_PARENT to PYTHONPATH for VPI"
                VPI_FOUND=true
                break
            fi
        fi
    done
fi

# Add VPI library paths to LD_LIBRARY_PATH
if [ -d /host/usr/lib/aarch64-linux-gnu ]; then
    if ls /host/usr/lib/aarch64-linux-gnu/libnvvpi* > /dev/null 2>&1; then
        export LD_LIBRARY_PATH="/host/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH"
        echo "Added /host/usr/lib/aarch64-linux-gnu to LD_LIBRARY_PATH for VPI"
    fi
fi

# Check if VPI is accessible
if python3 -c "import vpi" 2>/dev/null; then
    echo "✓ VPI is accessible in container"
else
    echo "⚠ VPI not accessible in container (will use CPU fallback)"
fi

# Suppress fontconfig warnings
export FONTCONFIG_FILE=/etc/fonts/fonts.conf 2>/dev/null || true
export FC_DEBUG=0 2>/dev/null || true

# Auto-start wheels driver node if not running
ROBOT_NAME=${VEHICLE_NAME:-robot1}
WHEELS_TOPIC="/${ROBOT_NAME}/wheels_driver_node/wheels_cmd"

echo "Checking if wheels driver node is ready..."
WHEELS_RUNNING=false

if rostopic list > /dev/null 2>&1; then
    if rostopic info "$WHEELS_TOPIC" > /dev/null 2>&1; then
        topic_info=$(rostopic info "$WHEELS_TOPIC" 2>/dev/null)
        subscriber_section=$(echo "$topic_info" | sed -n '/Subscribers:/,/Publishers:/p' | head -n -1)
        subscriber_count=$(echo "$subscriber_section" | tail -n +2 | grep -v "^$" | grep -v "^Publishers:" | wc -l)
        
        if [ "$subscriber_count" -gt 0 ] 2>/dev/null; then
            echo "✓ wheels driver topic has $subscriber_count subscriber(s)"
            WHEELS_RUNNING=true
        else
            if rosnode list 2>/dev/null | grep -q "wheels_driver_node"; then
                echo "⚠ wheels_driver_node is running but topic has no subscribers"
            fi
        fi
    fi
fi

if [ "$WHEELS_RUNNING" = false ]; then
    echo "⚠ Wheels driver not ready - you may need to start it manually"
    echo "  roslaunch duckietown wheels_driver.launch veh:=$ROBOT_NAME"
fi

# Execute the command
echo "Starting application: $@"
exec "$@"

