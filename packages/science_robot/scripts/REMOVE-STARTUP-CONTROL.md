# Removing Robot Startup Control Server (Port 5001)

This document explains how to completely remove the host-level startup control server if you no longer need it.

## Quick Removal

Run the uninstall script:
```bash
cd packages/science_robot/scripts
sudo ./uninstall-host-control.sh
```

## Manual Removal Steps

If you prefer to remove manually:

### 1. Stop and Disable Systemd Service

```bash
sudo systemctl stop robot-startup-control.service
sudo systemctl disable robot-startup-control.service
sudo systemctl daemon-reload
```

### 2. Remove Systemd Service File

```bash
sudo rm /etc/systemd/system/robot-startup-control.service
sudo systemctl daemon-reload
```

### 3. Remove Created Files

Delete these files from `packages/science_robot/scripts/`:
- `startup-control-server.py`
- `requirements-host.txt`
- `install-host-control.sh`
- `uninstall-host-control.sh`
- `REMOVE-STARTUP-CONTROL.md` (this file)

### 4. (Optional) Remove Python Dependencies

Only if Flask and psutil are not needed elsewhere:
```bash
pip3 uninstall flask psutil
# Or if installed system-wide:
sudo pip3 uninstall flask psutil
```

### 5. Verify Removal

Check that the service is gone:
```bash
systemctl status robot-startup-control.service  # Should show "not found"
```

Check that port 5001 is free:
```bash
netstat -tuln | grep 5001  # Should show nothing
```

## What This Removes

This removes:
- ✅ Port 5001 startup control web server
- ✅ Systemd service for auto-start
- ✅ Host-level installation files

This does NOT remove:
- ❌ Port 5000 robot web interface (runs in Docker)
- ❌ Robot control panel in main UI (in web_server.py)
- ❌ Robot control functionality in Docker container

## Notes

- The robot web interface on port 5000 (in Docker) will continue to work
- Robot control functionality in the main UI is unaffected
- Only the host-level startup control server is removed

