# Testing Collision Avoidance & Enhanced Emergency Stop

This guide covers testing the new collision avoidance and enhanced emergency stop features.

## Quick Syntax Check (No Docker Required)

```bash
cd science-robot-v2
python3 -m py_compile packages/science_robot/src/science_robot/collision_avoidance.py
python3 -m py_compile packages/science_robot/src/science_robot/motor_controller.py
python3 -m py_compile packages/science_robot/src/science_robot/config.py
```

## Full Test Suite (Requires Docker)

### Prerequisites
1. Docker daemon running
2. Docker image built: `docker build -t science-robot-v2:latest .`

### Run Tests

```bash
./test-collision-avoidance.sh
```

This will test:
1. ✅ Syntax validation
2. ✅ Module imports
3. ✅ Configuration validation
4. ✅ Emergency stop topic configuration
5. ✅ Collision avoidance module structure
6. ✅ Motor controller emergency stop methods

## Runtime Testing

### 1. Test Enhanced Emergency Stop

**Via Web Interface:**
```bash
./run-with-web.sh
```

Then:
- Open web browser to `http://<robot-ip>:5000`
- Click the **"🛑 Emergency Stop"** button
- Verify:
  - Motors stop immediately
  - Emergency stop status appears in dashboard
  - No motor commands are accepted until cleared

**Via Terminal:**
```bash
# In the container, press 's' key for emergency stop
# Or use ROS topic:
rostopic pub /robot1/wheels_driver_node/emergency_stop std_msgs/Header "{}"
```

**Verify Emergency Stop:**
```bash
# Check if emergency stop flag is set
docker exec <container> rostopic echo /robot1/wheels_driver_node/wheels_cmd
# Should show zero velocities
```

### 2. Test Collision Avoidance

#### Test with ToF Sensors (if available)

1. **Verify ToF sensors are publishing:**
```bash
rostopic list | grep tof
rostopic echo /robot1/tof_node/distance
```

2. **Run robot with collision avoidance:**
```bash
./run-with-web.sh
```

3. **Test scenarios:**
   - **Emergency zone (< 0.15m)**: Place obstacle very close
     - Expected: Immediate stop, emergency status
   - **Warning zone (0.15-0.30m)**: Place obstacle at medium distance
     - Expected: Speed reduction, warning status
   - **Safe zone (> 0.50m)**: Clear path
     - Expected: Normal operation

#### Test with Video-Only (no ToF sensors)

Collision avoidance will automatically fall back to video-based detection:

1. **Run robot:**
```bash
./run-with-web.sh
```

2. **Monitor web dashboard:**
   - Check "Collision Risk" status
   - Check "Distance" reading
   - Watch video feed for obstacle overlays

3. **Test scenarios:**
   - Place obstacle in front of camera
   - Verify edge detection triggers
   - Check distance estimates

### 3. Test Speed Reduction

When collision is detected in warning zone:

1. Start robot tracking a wave
2. Place obstacle at ~0.25m distance
3. Verify:
   - Robot slows down (check motor speeds in dashboard)
   - Warning status appears
   - Robot doesn't stop completely (unless in emergency zone)

### 4. Test Multi-Sensor Fusion

If both ToF and video detect obstacles:

1. Enable both sensors
2. Place obstacle in front
3. Verify:
   - Both sensors report obstacle
   - Fused distance is calculated
   - Higher confidence in detection

## Configuration Testing

### Test Different Safety Distances

```bash
# More conservative (larger safety zones)
docker run ... \
  -e "COLLISION_EMERGENCY_DISTANCE=0.20" \
  -e "COLLISION_WARNING_DISTANCE=0.40" \
  -e "COLLISION_SAFE_DISTANCE=0.60" \
  science-robot-v2:latest

# Less conservative (smaller safety zones)
docker run ... \
  -e "COLLISION_EMERGENCY_DISTANCE=0.10" \
  -e "COLLISION_WARNING_DISTANCE=0.20" \
  -e "COLLISION_SAFE_DISTANCE=0.30" \
  science-robot-v2:latest
```

### Disable Collision Avoidance

```bash
docker run ... \
  -e "ENABLE_COLLISION_AVOIDANCE=False" \
  science-robot-v2:latest
```

## Visual Verification

### Check Video Overlay

1. Enable display output or web interface
2. Verify overlay shows:
   - **ToF distance** (if available)
   - **Video detection** (if obstacle detected)
   - **Risk level** (none/warning/emergency)
   - **Distance bar** (visual indicator)
   - **Obstacle position** (circle on frame)

### Check Web Dashboard

1. Open web interface
2. Verify status panel shows:
   - **Collision Risk**: none/warning/emergency
   - **Distance**: distance in meters
   - Color coding:
     - Green: Safe
     - Yellow: Warning
     - Red: Emergency

## Troubleshooting

### Collision Avoidance Not Working

1. **Check if enabled:**
```bash
docker exec <container> python3 -c "from science_robot import config; print(config.ENABLE_COLLISION_AVOIDANCE)"
```

2. **Check ToF sensors:**
```bash
rostopic list | grep tof
rostopic echo /robot1/tof_node/distance
```

3. **Check logs:**
```bash
docker logs <container> | grep -i collision
```

### Emergency Stop Not Working

1. **Check emergency stop topic:**
```bash
rostopic info /robot1/wheels_driver_node/emergency_stop
```

2. **Check motor controller:**
```bash
docker logs <container> | grep -i "emergency stop"
```

3. **Verify wheels_driver_node is running:**
```bash
rosnode list | grep wheels_driver
```

## Expected Behavior

### Emergency Stop
- ✅ Motors stop immediately
- ✅ Emergency stop topic is published
- ✅ Motor commands are blocked
- ✅ Status shows "EMERGENCY_STOP"
- ✅ Can be cleared manually (if implemented)

### Collision Avoidance
- ✅ Detects obstacles via ToF (if available)
- ✅ Detects obstacles via video (always)
- ✅ Fuses sensor data when both available
- ✅ Stops in emergency zone
- ✅ Slows in warning zone
- ✅ Normal operation in safe zone
- ✅ Visual feedback on video/web

## Success Criteria

✅ All syntax checks pass
✅ Modules import successfully
✅ Configuration is valid
✅ Emergency stop stops motors immediately
✅ Collision avoidance detects obstacles
✅ Speed reduction works in warning zone
✅ Visual overlays appear correctly
✅ Web dashboard shows collision status

## Next Steps

After successful testing:
1. Calibrate distance thresholds for your environment
2. Adjust speed reduction factor if needed
3. Fine-tune edge detection thresholds
4. Test in real-world scenarios
5. Monitor performance and adjust as needed

