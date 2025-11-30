# Robot Startup Control System

This directory contains the host-level startup control server that runs independently of the robot's Docker container.

## Architecture Overview

### Two Web Servers:

1. **Port 5000 - Robot Web Interface** (Docker container)
   - Full robot monitoring and control UI
   - Runs inside Docker container when robot is active
   - Includes video feed, tuning panels, robot control
   - Only available when robot container is running

2. **Port 5001 - Startup Control Server** (Host systemd service)
   - Lightweight startup/shutdown control
   - Always running on host (via systemd)
   - Simple interface to start/stop robot Docker container
   - Available even when robot is off

## Files

### Startup Control Server (Port 5001):
- **`startup-control-server.py`** - Standalone Flask server for Docker container management
- **`requirements-host.txt`** - Minimal dependencies (Flask, psutil only)
- **`robot-startup-control.service`** - Systemd service file
- **`install-host-control.sh`** - Installation script
- **`uninstall-host-control.sh`** - Removal script

### Removal/Undo Documentation:
- **`REMOVE-STARTUP-CONTROL.md`** - Detailed removal instructions
- **`STARTUP-CONTROL-README.md`** - This file

## Installation

### On the Robot Host:

1. **Install Python dependencies:**
   ```bash
   cd packages/science_robot/scripts
   pip3 install --user -r requirements-host.txt
   ```

2. **Install systemd service:**
   ```bash
   sudo ./install-host-control.sh
   ```

3. **Start and enable service:**
   ```bash
   sudo systemctl start robot-startup-control.service
   sudo systemctl enable robot-startup-control.service
   ```

4. **Access control panel:**
   - Open browser to: `http://robot1.local:5001` or `http://<robot-ip>:5001`

## Usage

### Starting the Robot:

1. Open `http://robot1.local:5001` in a browser
2. Click "▶️ Start Robot" button
3. Wait a few seconds for Docker container to start
4. Robot web interface will be available at `http://robot1.local:5000`

### Stopping the Robot:

1. From port 5001: Click "⏸️ Stop Robot"
2. Or from port 5000: Click "⏹️ Quit" in the robot web interface

### Shutdown:

- Click "🔄 Shutdown Robot" on port 5001 for graceful shutdown

## How It Works

### Container Detection:
- Uses `docker ps` to check if robot container is running
- Looks for containers using `science-robot-v2:latest` image

### Starting Robot:
- Executes `run-with-web.sh` script in background
- Script launches Docker container with robot software
- Container runs on port 5000 (robot web interface)

### Stopping Robot:
- Finds running container by image name
- Stops container gracefully using `docker stop`
- Optionally attempts ROS shutdown before stopping

## Easy Removal

To completely remove this system:

```bash
sudo ./uninstall-host-control.sh
```

Or follow manual steps in `REMOVE-STARTUP-CONTROL.md`

## Dependencies

**Host system only needs:**
- Python 3
- Flask (web server)
- psutil (process management, optional)

**No dependency on:**
- Robot Docker container
- ROS
- OpenCV
- MediaPipe
- Any robot-specific code

## Configuration

Environment variables (set in systemd service or script):
- `STARTUP_CONTROL_PORT=5001` - Port for startup control server
- `STARTUP_CONTROL_HOST=0.0.0.0` - Host to bind to
- `VEHICLE_NAME=robot1` - Robot name
- `ROS_MASTER_URI=http://localhost:11311` - ROS master URI
- `WEB_SERVER_PORT=5000` - Port for robot web interface

## Troubleshooting

### Service won't start:
```bash
sudo systemctl status robot-startup-control.service
sudo journalctl -u robot-startup-control.service -f
```

### Port 5001 already in use:
- Check what's using it: `sudo netstat -tuln | grep 5001`
- Or change port in service file and restart

### Docker commands failing:
- Ensure Docker is installed and running: `sudo systemctl status docker`
- Ensure user has Docker permissions (or run service as root)

### Can't start robot:
- Check Docker is running
- Verify `run-with-web.sh` exists and is executable
- Check logs: `sudo journalctl -u robot-startup-control.service`

## Files Modified/Created

### New Files (can be easily deleted):
- `startup-control-server.py`
- `requirements-host.txt`
- `install-host-control.sh`
- `uninstall-host-control.sh`
- `REMOVE-STARTUP-CONTROL.md`
- `STARTUP-CONTROL-README.md`
- `robot-startup-control.service` (updated for port 5001)

### Existing Files (NOT modified):
- `web_server.py` - Robot control panel in main UI (port 5000) remains unchanged
- `requirements.txt` - Docker container requirements unchanged
- All robot code remains unchanged

