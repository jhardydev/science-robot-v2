# Post-Build Testing Guide

This guide provides step-by-step testing procedures after successfully building the Docker image.

## Prerequisites

Before testing, ensure:
- [ ] Docker image built successfully: `science-robot-v2:latest`
- [ ] ROS master is running on the robot
- [ ] Camera node is running and publishing images
- [ ] Wheels driver node is running (or can be started)

## Testing Steps

### Step 1: Verify Docker Image

```bash
# Check image exists
docker images | grep science-robot-v2

# Expected output: science-robot-v2:latest with recent timestamp
```

### Step 2: Test Container Startup (Basic)

```bash
# Test container can start and entrypoint runs
docker run -it --rm --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e VEHICLE_NAME=robot1 \
  science-robot-v2:latest bash

# Inside container, verify:
# - ROS is sourced: echo $ROS_MASTER_URI
# - Catkin workspace is sourced: rospack find science_robot
# - Python package is importable: python3 -c "from science_robot import config; print('OK')"
```

**Expected Results:**
- Container starts without errors
- ROS environment variables are set
- `rospack find science_robot` returns `/code/packages/src/science_robot` or similar
- Python import succeeds

### Step 3: Verify ROS Package Discovery

```bash
# Test ROS package is discoverable
docker run -it --rm --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  science-robot-v2:latest \
  bash -c "source /opt/ros/noetic/setup.bash && \
           source /code/packages/devel/setup.bash && \
           rospack find science_robot"

# Expected: Returns path to science_robot package
```

**Expected Output:**
```
/code/packages/src/science_robot
```
or
```
/code/packages/devel/../src/science_robot
```

### Step 4: Test ROS Launch File

```bash
# Test launch file syntax (dry run)
docker run -it --rm --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e VEHICLE_NAME=robot1 \
  science-robot-v2:latest \
  roslaunch science_robot science_robot.launch robot_name:=robot1 --check

# Expected: No errors, just validates launch file
```

**Expected Results:**
- Launch file validates without XML errors
- No "Invalid <arg> tag" errors

### Step 5: Test ROS Node Startup (Without Running)

```bash
# Start container and verify node can initialize
docker run -it --rm --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e VEHICLE_NAME=robot1 \
  science-robot-v2:latest \
  timeout 10 roslaunch science_robot science_robot.launch robot_name:=robot1 || true

# Check logs for:
# - "Initializing Duckiebot Science Fair Robot v2.0"
# - "Camera subscriber initialized"
# - "Motor controller initialized"
# - No import errors
```

**Expected Results:**
- Node starts without Python import errors
- ROS node initializes
- Camera subscriber connects (may timeout waiting for frames - that's OK)
- Motor controller initializes
- No MediaPipe errors (if MediaPipe not available, should handle gracefully)

### Step 6: Verify ROS Topic Connectivity

```bash
# Start container in background
docker run -d --name test-science-robot --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e VEHICLE_NAME=robot1 \
  science-robot-v2:latest

# Wait a few seconds for startup
sleep 5

# Check if node is running
docker exec test-science-robot rosnode list | grep science_robot

# Check if topics are being used
docker exec test-science-robot rostopic list | grep -E "(camera|wheels)"

# Check motor command topic
docker exec test-science-robot rostopic info /robot1/wheels_driver_node/wheels_cmd

# Cleanup
docker stop test-science-robot
docker rm test-science-robot
```

**Expected Results:**
- `science_robot_controller` node appears in `rosnode list`
- Camera topic `/robot1/camera_node/image/compressed` is subscribed
- Motor topic `/robot1/wheels_driver_node/wheels_cmd` is being published to

### Step 7: Test Camera Frame Reception

```bash
# Start container and check camera frames
docker run -it --rm --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e VEHICLE_NAME=robot1 \
  science-robot-v2:latest \
  bash -c "python3 -c \"
import rospy
from sensor_msgs.msg import CompressedImage
import time

rospy.init_node('test_camera')
received = False

def callback(msg):
    global received
    received = True
    print('Camera frame received!')

sub = rospy.Subscriber('/robot1/camera_node/image/compressed', CompressedImage, callback)
timeout = time.time() + 10
while not received and time.time() < timeout:
    rospy.sleep(0.1)

if received:
    print('SUCCESS: Camera frames are being received')
else:
    print('WARNING: No camera frames received in 10 seconds')
\""
```

**Expected Results:**
- Camera frames are received within 10 seconds
- No subscription errors

### Step 8: Test Motor Command Publishing

```bash
# Test motor commands can be published
docker run -it --rm --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e VEHICLE_NAME=robot1 \
  science-robot-v2:latest \
  bash -c "python3 -c \"
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Header

rospy.init_node('test_motor')
pub = rospy.Publisher('/robot1/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
rospy.sleep(1)

msg = WheelsCmdStamped()
msg.header = Header()
msg.header.stamp = rospy.Time.now()
msg.vel_left = 0.0
msg.vel_right = 0.0

pub.publish(msg)
print('SUCCESS: Motor command published')
print('Subscribers:', pub.get_num_connections())
\""
```

**Expected Results:**
- Motor command publishes without errors
- Ideally has at least 1 subscriber (wheels_driver_node)

### Step 9: Full Integration Test (Short Run)

```bash
# Run the robot for 30 seconds to test full integration
docker run -it --rm --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e VEHICLE_NAME=robot1 \
  science-robot-v2:latest \
  timeout 30 roslaunch science_robot science_robot.launch robot_name:=robot1 || true

# Check logs for:
# - Successful initialization
# - Camera frames being processed
# - No crashes or errors
```

**Expected Results:**
- Robot initializes successfully
- Main loop runs without errors
- Processes camera frames
- No crashes or exceptions

### Step 10: Test Error Handling

```bash
# Test with wrong robot name (should handle gracefully)
docker run -it --rm --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e VEHICLE_NAME=wrong_robot \
  science-robot-v2:latest \
  timeout 10 roslaunch science_robot science_robot.launch robot_name:=wrong_robot || true

# Should show warnings about missing topics but not crash
```

**Expected Results:**
- Handles missing topics gracefully
- Shows warnings but doesn't crash
- Continues running (may not function correctly, but doesn't error out)

## Quick Test Script

Save this as `test-container.sh`:

```bash
#!/bin/bash
set -e

echo "=== Testing Science Robot v2.0 Container ==="

# Test 1: Image exists
echo "Test 1: Checking Docker image..."
docker images | grep -q science-robot-v2 || { echo "ERROR: Image not found"; exit 1; }
echo "✓ Image exists"

# Test 2: Container starts
echo "Test 2: Testing container startup..."
docker run --rm --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e VEHICLE_NAME=robot1 \
  science-robot-v2:latest \
  bash -c "source /opt/ros/noetic/setup.bash && \
           source /code/packages/devel/setup.bash && \
           rospack find science_robot > /dev/null && \
           python3 -c 'from science_robot import config; print(\"OK\")'" || { echo "ERROR: Container test failed"; exit 1; }
echo "✓ Container starts and package is accessible"

# Test 3: Launch file validates
echo "Test 3: Validating launch file..."
docker run --rm --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  science-robot-v2:latest \
  roslaunch science_robot science_robot.launch --check > /dev/null 2>&1 || { echo "ERROR: Launch file invalid"; exit 1; }
echo "✓ Launch file is valid"

echo ""
echo "=== All Basic Tests Passed ==="
echo "Ready for full integration testing on robot"
```

Make it executable:
```bash
chmod +x test-container.sh
./test-container.sh
```

## Success Criteria

✅ **Container builds successfully**
✅ **Container starts without errors**
✅ **ROS package is discoverable**
✅ **Launch file validates**
✅ **Python imports work**
✅ **ROS node can initialize**
✅ **Camera topic connectivity works**
✅ **Motor topic connectivity works**
✅ **No critical errors in logs**

## Troubleshooting

### If ROS package not found:
- Check Catkin workspace was built: `ls /code/packages/devel/setup.bash`
- Source workspace manually: `source /code/packages/devel/setup.bash`

### If camera not working:
- Verify camera node is running: `rostopic list | grep camera`
- Check topic name matches robot name: `rostopic echo /robot1/camera_node/image/compressed`

### If motors not working:
- Verify wheels_driver_node is running: `rosnode list | grep wheels`
- Check topic has subscribers: `rostopic info /robot1/wheels_driver_node/wheels_cmd`

### If MediaPipe errors:
- This is expected on ARM64 - robot will run without gesture detection
- Check logs for "MediaPipe not available" message (this is OK)

## Next Steps After Testing

Once all tests pass:
1. Run full integration test with actual robot hardware
2. Test wave detection functionality
3. Test gesture recognition (if MediaPipe available)
4. Test motor control and navigation
5. Test dance routines
6. Monitor performance and adjust settings in `config.py`

