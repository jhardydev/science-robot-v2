# Integration Testing Guide for Science Robot v2.0

This guide provides step-by-step procedures for testing the robot's full functionality on actual hardware.

## Prerequisites

- [ ] Docker image built and tested: `science-robot-v2:latest`
- [ ] All unit tests passing: `./test-all.sh full`
- [ ] ROS master running on robot
- [ ] Camera node running and publishing images
- [ ] Wheels driver node running (or can be started)
- [ ] VPI libraries mounted (for GPU acceleration)
- [ ] Robot has sufficient battery/power
- [ ] Safe testing area (robot won't fall off table, etc.)

## Integration Test Checklist

### Phase 1: Basic Functionality

#### 1.1 Container Startup and ROS Connection
- [ ] Container starts without errors
- [ ] ROS master connection successful
- [ ] Camera topic subscription works
- [ ] Motor topic publisher connects
- [ ] VPI acceleration enabled (if available)

**Test Command:**
```bash
docker run -it --rm --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e VEHICLE_NAME=robot1 \
  -v /usr/lib/python3/dist-packages/vpi:/host/usr/lib/python3/dist-packages/vpi:ro \
  -v /usr/lib/aarch64-linux-gnu:/host/usr/lib/aarch64-linux-gnu:ro \
  science-robot-v2:latest
```

**Success Criteria:**
- Container starts and shows initialization messages
- No import errors
- ROS topics are subscribed/published
- VPI status shows as accessible (if available)

#### 1.2 Camera Frame Reception
- [ ] Camera frames are being received
- [ ] Frame rate is acceptable (>5 FPS)
- [ ] Image quality is good
- [ ] No frame drops or errors

**Test Command:**
```bash
# Check camera topic
rostopic hz /robot1/camera_node/image/compressed

# View camera feed (if display available)
rostopic echo /robot1/camera_node/image/compressed --noarr
```

**Success Criteria:**
- Camera frames received at >5 FPS
- No errors in camera subscription
- Images are valid and not corrupted

#### 1.3 Motor Control
- [ ] Motor commands can be published
- [ ] Wheels respond to commands
- [ ] Emergency stop works ('s' key)
- [ ] No motor errors

**Test Command:**
```bash
# Test motor command publishing
rostopic pub /robot1/wheels_driver_node/wheels_cmd duckietown_msgs/WheelsCmdStamped \
  "header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  vel_left: 0.1
  vel_right: 0.1" -1
```

**Success Criteria:**
- Motor commands accepted
- Wheels move (or attempt to move)
- No errors in motor controller

### Phase 2: Core Features

#### 2.1 Wave Detection
- [ ] Wave detection initializes
- [ ] Detects waving motion
- [ ] Tracks waving person
- [ ] Robot steers toward detected wave

**Test Procedure:**
1. Start robot container
2. Stand in front of robot
3. Wave hand side-to-side
4. Observe robot behavior

**Success Criteria:**
- Robot detects waving motion
- Robot turns toward waving person
- Smooth tracking without jitter
- No false positives

#### 2.2 Gesture Recognition (if MediaPipe available)
- [ ] Gesture detector initializes
- [ ] Hand detection works
- [ ] Open hand gesture recognized (dance trigger)
- [ ] Gesture hold time tracked correctly

**Test Procedure:**
1. Start robot container
2. Show open hand (all fingers extended)
3. Hold for 1+ seconds
4. Observe dance routine trigger

**Success Criteria:**
- Hand detected reliably
- Open hand gesture recognized
- Dance routine triggers after 1 second hold
- No false triggers

#### 2.3 Dance Routines
- [ ] Dance routine executes when triggered
- [ ] Multiple dance moves work
- [ ] Robot returns to idle after dance
- [ ] No crashes during dance

**Test Procedure:**
1. Trigger dance routine (gesture or manual)
2. Observe dance sequence
3. Verify robot returns to idle

**Success Criteria:**
- Dance routine executes smoothly
- All dance moves work
- Robot returns to normal operation
- No motor errors or crashes

### Phase 3: Performance and Reliability

#### 3.1 Performance Metrics
- [ ] Frame processing rate >5 FPS
- [ ] CPU usage acceptable (<80%)
- [ ] Memory usage stable
- [ ] No memory leaks

**Test Command:**
```bash
# Monitor resource usage
docker stats <container_name>

# Check frame rate in logs
docker logs <container_name> | grep -i fps
```

**Success Criteria:**
- Processing rate >5 FPS
- CPU usage <80% average
- Memory stable (no continuous growth)
- No performance degradation over time

#### 3.2 Long-Run Stability
- [ ] Robot runs for 10+ minutes without crashes
- [ ] No memory leaks
- [ ] Error recovery works
- [ ] Graceful degradation on errors

**Test Procedure:**
1. Start robot
2. Let it run for 10+ minutes
3. Monitor logs for errors
4. Test error scenarios (disconnect camera, etc.)

**Success Criteria:**
- No crashes in 10+ minute run
- Memory usage stable
- Errors handled gracefully
- Robot continues operating after errors

#### 3.3 Error Handling
- [ ] Camera disconnection handled gracefully
- [ ] ROS master disconnection handled
- [ ] Invalid commands don't crash robot
- [ ] Recovery after errors works

**Test Scenarios:**
1. Stop camera node while robot running
2. Restart ROS master
3. Send invalid motor commands
4. Network interruptions

**Success Criteria:**
- Errors logged but don't crash robot
- Robot recovers when services restored
- Graceful degradation (continues with reduced functionality)

### Phase 4: Real-World Scenarios

#### 4.1 Science Fair Demo
- [ ] Complete demo sequence works
- [ ] Wave detection → steering → gesture → dance
- [ ] Multiple people can interact
- [ ] Demo runs for 5+ minutes

**Test Procedure:**
1. Full demo sequence
2. Multiple interactions
3. Extended operation

**Success Criteria:**
- Complete demo works end-to-end
- Handles multiple interactions
- Stable for extended periods

#### 4.2 Edge Cases
- [ ] Works in different lighting conditions
- [ ] Handles multiple people in frame
- [ ] Works at different distances
- [ ] Handles rapid gestures

**Test Scenarios:**
- Bright/dim lighting
- Multiple people
- Close/far distances
- Fast gesture changes

## Test Execution Log

### Date: ___________
### Tester: ___________
### Robot: ___________

#### Phase 1 Results:
- Container Startup: [ ] Pass [ ] Fail [ ] N/A
- Camera Reception: [ ] Pass [ ] Fail [ ] N/A
- Motor Control: [ ] Pass [ ] Fail [ ] N/A

**Notes:**
```
[Add notes here]
```

#### Phase 2 Results:
- Wave Detection: [ ] Pass [ ] Fail [ ] N/A
- Gesture Recognition: [ ] Pass [ ] Fail [ ] N/A
- Dance Routines: [ ] Pass [ ] Fail [ ] N/A

**Notes:**
```
[Add notes here]
```

#### Phase 3 Results:
- Performance: [ ] Pass [ ] Fail [ ] N/A
- Stability: [ ] Pass [ ] Fail [ ] N/A
- Error Handling: [ ] Pass [ ] Fail [ ] N/A

**Notes:**
```
[Add notes here]
```

#### Phase 4 Results:
- Demo Sequence: [ ] Pass [ ] Fail [ ] N/A
- Edge Cases: [ ] Pass [ ] Fail [ ] N/A

**Notes:**
```
[Add notes here]
```

## Troubleshooting

### Camera Not Working
- Verify camera node: `rosnode list | grep camera`
- Check topic: `rostopic echo /robot1/camera_node/image/compressed`
- Restart camera node if needed

### Motors Not Responding
- Check wheels_driver_node: `rosnode list | grep wheels`
- Verify topic subscribers: `rostopic info /robot1/wheels_driver_node/wheels_cmd`
- Check FSM mode allows external commands

### Wave Detection Not Working
- Check camera frames are being received
- Verify processing rate in logs
- Adjust detection thresholds in config.py if needed

### Gesture Recognition Not Working
- Check if MediaPipe is available: `python3 -c "import mediapipe; print('OK')"`
- Verify hand is clearly visible
- Check gesture confidence thresholds

## Next Steps After Integration Testing

1. Document any issues found
2. Adjust configuration parameters if needed
3. Optimize performance based on test results
4. Prepare for science fair demonstration
5. Create user guide for operation

