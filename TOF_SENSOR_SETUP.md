# ToF Sensor Setup Guide

Your ToF sensor is configured at:
- **I2C Bus**: 1
- **I2C Channel**: 6  
- **I2C Address**: 0x29 (likely VL53L0X sensor)

## Verify ToF Sensor is Working

**Note:** ROS commands must be run inside the container or connected to the container's ROS master. Use the helper scripts provided.

### Quick Check (Recommended)

```bash
# Run the diagnostic script (works from host or container)
./check-tof-sensor.sh
```

### Manual Checks

#### Option 1: Use helper script (from host)

```bash
# Run any ROS command inside the container
./ros-commands.sh "rostopic list | grep tof"
./ros-commands.sh "rostopic echo /robot1/tof_node/distance"
```

#### Option 2: Run commands inside container

```bash
# Get container name
docker ps | grep science-robot-v2

# Execute ROS commands inside container
docker exec -it <container-name> bash -c "source /opt/ros/noetic/setup.bash && rostopic list | grep tof"
```

#### Option 3: Connect host ROS to container ROS master

```bash
# On host, set ROS_MASTER_URI to point to container
export ROS_MASTER_URI=http://robot1:11311  # or container IP
source /opt/ros/noetic/setup.bash  # if ROS installed on host

# Now ROS commands work from host
rostopic list | grep tof
```

### 1. Check if ToF sensor node is running

```bash
./ros-commands.sh "rosnode list | grep tof"
```

### 2. Check available ToF topics

```bash
./ros-commands.sh "rostopic list | grep -i tof"
./ros-commands.sh "rostopic list | grep -i distance"
./ros-commands.sh "rostopic list | grep -i range"
```

### 3. Monitor ToF sensor data

```bash
# Try common topic names
./ros-commands.sh "rostopic echo /robot1/tof_node/distance"
./ros-commands.sh "rostopic echo /robot1/tof_node/range"
./ros-commands.sh "rostopic echo /robot1/tof_0x29/distance"

# Check topic frequency
./ros-commands.sh "rostopic hz /robot1/tof_node/distance"
```

### 4. Check topic message type

```bash
./ros-commands.sh "rostopic type /robot1/tof_node/distance"
# Should show: sensor_msgs/Range
```

## Start ToF Sensor Node (if not running)

If the ToF sensor node isn't running, you may need to start it:

```bash
# Duckietown ToF node (if available)
roslaunch duckietown tof_node.launch veh:=robot1

# Or check for other ToF launch files
roslaunch duckietown tof.launch veh:=robot1
```

## Manual Topic Configuration

If auto-detection doesn't find your ToF sensor topic, you can manually specify it:

```bash
# Set environment variable before running
export TOF_TOPIC_OVERRIDE="/robot1/tof_node/distance"

# Or in docker run command
docker run ... -e TOF_TOPIC_OVERRIDE="/robot1/tof_node/distance" ...
```

## Verify Collision Avoidance is Using ToF

Check the robot logs for:

```
✓ Subscribed to ToF front sensor: /robot1/tof_node/distance
Collision avoidance system initialized
  Emergency zone: 0.15m
  Warning zone: 0.3m
  Safe zone: 0.5m
```

If you see:
```
No ToF sensors found - using video-based detection only
```

Then the ToF sensor topic is not being found. Check:
1. Is the ToF node running?
2. What topic is it publishing to?
3. Use `TOF_TOPIC_OVERRIDE` to manually specify the topic

## I2C Address Configuration

The collision avoidance system will automatically detect ToF sensors, but if your sensor uses a different topic naming convention, you can:

1. **Find the actual topic name:**
   ```bash
   rostopic list | grep -i tof
   ```

2. **Set manual override:**
   ```bash
   export TOF_TOPIC_OVERRIDE="/actual/topic/name"
   ```

3. **Or update the topic list in collision_avoidance.py** to include your specific topic pattern

## Testing ToF Sensor

1. **Check sensor is publishing:**
   ```bash
   rostopic echo /robot1/tof_node/distance -n 10
   ```

2. **Verify distance readings:**
   - Place object at known distance (e.g., 0.5m)
   - Check if readings match expected values
   - Readings should be in meters

3. **Test collision avoidance:**
   - Move object closer than 0.15m → Emergency stop
   - Move object to 0.15-0.30m → Warning (speed reduction)
   - Move object beyond 0.50m → Normal operation

## Troubleshooting

### ToF sensor not detected

1. **Check I2C connection:**
   ```bash
   i2cdetect -y 1
   # Should show 0x29 in the output
   ```

2. **Check ROS node:**
   ```bash
   rosnode list
   rosnode info /robot1/tof_node
   ```

3. **Check topic exists:**
   ```bash
   rostopic list
   rostopic info /robot1/tof_node/distance
   ```

4. **Check message type:**
   ```bash
   rostopic type /robot1/tof_node/distance
   # Should be sensor_msgs/Range
   ```

### Distance readings seem wrong

- Check sensor calibration
- Verify sensor is pointing in correct direction
- Check for obstructions or reflections
- Verify sensor range (VL53L0X typically 0-2m)

### Collision avoidance not using ToF

- Check logs for "Subscribed to ToF front sensor"
- Verify topic is publishing: `rostopic hz /robot1/tof_node/distance`
- Check if `TOF_TOPIC_OVERRIDE` is set correctly
- Review collision avoidance initialization logs

