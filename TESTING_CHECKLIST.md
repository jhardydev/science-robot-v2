# Testing Checklist for Science Robot v2.0

## Pre-Testing Verification

### ✅ Code Structure
- [x] All source files migrated with correct imports
- [x] Catkin package structure in place
- [x] ROS launch file created
- [x] Dockerfile updated for Ubuntu 22.04

### ⚠️ Items to Verify Before Testing

#### 1. Docker Base Image
- **Current**: `duckietown/dt-ros-commons:ente-arm64v8`
- **Action**: Verify this is the correct base image for Ubuntu 22.04
- **Note**: Original v1 used `daffy-arm64v8` for Ubuntu 20.04
- **Check**: Confirm "ente" tag supports Ubuntu 22.04 and ROS Noetic

#### 2. Catkin Workspace Build
- **Issue**: Dockerfile uses `|| true` which continues even if build fails
- **Location**: Line 74 in Dockerfile
- **Action**: Monitor build logs to ensure Catkin workspace builds successfully
- **Fallback**: Entrypoint script will attempt to build if not already built

#### 3. Package Path Resolution
- **Code**: `config.py` uses `rospkg.RosPack()` to get package path
- **Fallback**: Has fallback path calculation if ROS package not found
- **Action**: Verify package is discoverable after Catkin build

#### 4. ROS Dependencies
- **Required**: `duckietown_msgs` must be available
- **Check**: Base image should include this, but verify at runtime

## Testing Steps

### 1. Build Docker Image
```bash
cd science-robot-v2
docker build -t science-robot-v2:latest .
```

**Watch for:**
- Catkin workspace build success
- All Python dependencies install correctly
- No errors during build

### 2. Verify Catkin Package
```bash
# Run with VPI mounting
docker run -it --rm \
  -v /usr/lib/python3/dist-packages/vpi:/host/usr/lib/python3/dist-packages/vpi:ro \
  -v /usr/lib/aarch64-linux-gnu:/host/usr/lib/aarch64-linux-gnu:ro \
  science-robot-v2:latest bash
# Inside container:
source /opt/ros/noetic/setup.bash
source /code/packages/devel/setup.bash  # or install/setup.bash
rospack find science_robot
# Test VPI: python3 -c "import vpi; print('VPI OK')" 2>/dev/null || echo "VPI not available (optional)"
```

**Expected**: Should return `/code/packages/src/science_robot`

### 3. Test ROS Launch
```bash
# On robot with ROS master running (with VPI mounting)
docker run -it --rm \
  --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e VEHICLE_NAME=robot1 \
  -v /usr/lib/python3/dist-packages/vpi:/host/usr/lib/python3/dist-packages/vpi:ro \
  -v /usr/lib/aarch64-linux-gnu:/host/usr/lib/aarch64-linux-gnu:ro \
  science-robot-v2:latest
```

**Watch for:**
- ROS node starts successfully
- Camera topic subscription works
- Motor topic publisher connects
- VPI acceleration enabled (if available) or graceful fallback message
- No import errors

### 4. Runtime Checks
- [ ] ROS master connection successful
- [ ] Camera frames received
- [ ] Motor commands can be published
- [ ] Gesture detection initializes
- [ ] VPI/CUDA acceleration enabled (check logs for "VPI acceleration enabled" or "VPI not accessible")

## Known Potential Issues

### Issue 1: Base Image Compatibility
**Risk**: `ente-arm64v8` might not be Ubuntu 22.04
**Solution**: If build fails, try:
- `duckietown/dt-ros-commons:daffy-arm64v8` (Ubuntu 20.04, should still work)
- Or check Duckietown docs for correct Ubuntu 22.04 image tag

### Issue 2: Catkin Build Fails Silently
**Risk**: Build failure masked by `|| true`
**Solution**: Check build logs carefully, entrypoint will retry

### Issue 3: Package Not Found
**Risk**: `rospack find science_robot` fails
**Solution**: Ensure workspace is sourced, check `ROS_PACKAGE_PATH`

## Quick Test Commands

```bash
# Test package discovery (with VPI mounting)
docker run -it --rm \
  -v /usr/lib/python3/dist-packages/vpi:/host/usr/lib/python3/dist-packages/vpi:ro \
  -v /usr/lib/aarch64-linux-gnu:/host/usr/lib/aarch64-linux-gnu:ro \
  science-robot-v2:latest bash -c \
  "source /opt/ros/noetic/setup.bash && \
   source /code/packages/devel/setup.bash && \
   rospack find science_robot"

# Test Python imports (with VPI mounting)
docker run -it --rm \
  -v /usr/lib/python3/dist-packages/vpi:/host/usr/lib/python3/dist-packages/vpi:ro \
  -v /usr/lib/aarch64-linux-gnu:/host/usr/lib/aarch64-linux-gnu:ro \
  science-robot-v2:latest python3 -c \
  "import sys; sys.path.insert(0, '/code/packages/src'); \
   from science_robot import config; print('Config loaded:', config.ROBOT_NAME)"

# Test launch file (with VPI mounting)
docker run -it --rm --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e VEHICLE_NAME=robot1 \
  -v /usr/lib/python3/dist-packages/vpi:/host/usr/lib/python3/dist-packages/vpi:ro \
  -v /usr/lib/aarch64-linux-gnu:/host/usr/lib/aarch64-linux-gnu:ro \
  science-robot-v2:latest \
  roslaunch science_robot science_robot.launch robot_name:=robot1
```

## Success Criteria

✅ Docker image builds without errors
✅ Catkin workspace builds successfully
✅ ROS package is discoverable
✅ Python imports work correctly
✅ ROS node starts and connects to topics
✅ Camera and motor topics are accessible

## If Issues Occur

1. **Build fails**: Check Dockerfile base image tag
2. **Package not found**: Verify Catkin workspace was built and sourced
3. **Import errors**: Check Python path includes package location
4. **ROS connection fails**: Verify ROS_MASTER_URI and network settings
5. **Topics not found**: Ensure robot's ROS nodes are running

