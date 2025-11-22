# X11 Display Setup for macOS (XQuartz)

This guide explains how to set up X11 forwarding from the robot to your macOS computer using XQuartz.

## Prerequisites

1. **Install XQuartz** (X11 server for macOS):
   ```bash
   # Using Homebrew:
   brew install --cask xquartz
   
   # Or download from: https://www.xquartz.org/
   ```

2. **Start XQuartz**:
   - Open XQuartz from Applications
   - Or run: `open -a XQuartz`

3. **Configure XQuartz** (one-time setup):
   - XQuartz → Preferences → Security
   - Check "Allow connections from network clients"
   - Restart XQuartz after changing this setting

## SSH Setup

### Option 1: SSH with X11 Forwarding (Recommended)

When connecting to the robot, use the `-X` or `-Y` flag:

```bash
# Basic X11 forwarding
ssh -X duckie@robot1

# Trusted X11 forwarding (less secure but more compatible)
ssh -Y duckie@robot1
```

### Option 2: Configure SSH Config (Persistent)

Add to `~/.ssh/config`:

```
Host robot1
    HostName robot1.local  # or IP address
    User duckie
    ForwardX11 yes
    ForwardX11Trusted yes
```

Then just use: `ssh robot1`

## Verify X11 Forwarding

Once connected via SSH, verify X11 is working:

```bash
# Check DISPLAY variable
echo $DISPLAY
# Should show something like: localhost:10.0

# Test with a simple X11 app
xeyes  # or xclock, xterm, etc.
```

If you see a window appear on your Mac, X11 forwarding is working!

## Running the Robot with Display

### Method 1: Using run-with-logs.sh with X11

```bash
# On the robot, after SSH with X11 forwarding:
export DISPLAY=:10.0  # or whatever $DISPLAY shows
./run-with-logs.sh --display
```

### Method 2: Manual Docker Run with X11

```bash
# On the robot, after SSH with X11 forwarding:
export DISPLAY=:10.0

# Get your IP from the robot's perspective
# (usually localhost or the Mac's IP on the network)
MAC_IP=$(echo $DISPLAY | cut -d: -f1)
DISPLAY_NUM=$(echo $DISPLAY | cut -d: -f2)

# Run container with X11 forwarding
docker run -it --rm --network host \
  -e ROS_MASTER_URI=http://localhost:11311 \
  -e VEHICLE_NAME=robot1 \
  -e DISPLAY=$MAC_IP$DISPLAY_NUM \
  -e DISPLAY_OUTPUT=true \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /tmp/science-robot-logs:/code/logs \
  -v /usr/lib/python3/dist-packages/vpi:/host/usr/lib/python3/dist-packages/vpi:ro \
  -v /usr/lib/aarch64-linux-gnu:/host/usr/lib/aarch64-linux-gnu:ro \
  science-robot-v2:latest
```

### Method 3: Using run-with-x11.sh (Helper Script)

See the `run-with-x11.sh` script for automated X11 setup.

## Troubleshooting

### "Cannot connect to X server"

1. **Check XQuartz is running**:
   ```bash
   # On Mac
   ps aux | grep -i xquartz
   ```

2. **Check DISPLAY variable**:
   ```bash
   # On robot (via SSH)
   echo $DISPLAY
   # Should not be empty
   ```

3. **Check X11 socket**:
   ```bash
   # On robot
   ls -la /tmp/.X11-unix/
   # Should see X0 or similar
   ```

4. **Test X11 connection**:
   ```bash
   # On robot
   xdpyinfo
   # Should show display information, not errors
   ```

### "X11 connection rejected"

1. **Enable network connections in XQuartz**:
   - XQuartz → Preferences → Security
   - Check "Allow connections from network clients"
   - Restart XQuartz

2. **Add your Mac's IP to xhost** (on Mac):
   ```bash
   # Get robot's IP
   ROBOT_IP=192.168.1.XXX  # replace with actual IP
   
   # Allow connections from robot
   xhost +$ROBOT_IP
   ```

### Display shows but is slow

- Use `-Y` (trusted forwarding) instead of `-X`
- Check network connection speed
- Consider using VNC instead (see VNC setup guide)

### No windows appear

1. **Check container logs**:
   ```bash
   docker logs <container-name>
   ```

2. **Verify DISPLAY_OUTPUT is enabled**:
   ```bash
   # In container
   echo $DISPLAY_OUTPUT
   # Should be "true"
   ```

3. **Check application logs**:
   ```bash
   find /tmp/science-robot-logs -name "*.log" | xargs tail -50
   ```

## Security Note

X11 forwarding can be a security risk. For production use, consider:
- Using VNC instead
- Using SSH tunnels
- Restricting X11 access with xhost

