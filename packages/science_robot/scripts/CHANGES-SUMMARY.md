# Port 5001 Startup Control - Changes Summary

## What Was Added

All new files for host-level startup control server (easily removable):

### New Files:
1. **`startup-control-server.py`** - Standalone Flask server (port 5001)
   - Manages Docker container lifecycle
   - Minimal dependencies (Flask, psutil)
   - Simple HTML interface

2. **`requirements-host.txt`** - Host-only Python dependencies
   - Flask>=2.0.0
   - psutil>=5.9.0

3. **`install-host-control.sh`** - Installation script
   - Installs Flask and psutil
   - Sets up systemd service
   - Configures for port 5001

4. **`uninstall-host-control.sh`** - Removal script
   - Stops and removes systemd service
   - Easy cleanup

5. **`STARTUP-CONTROL-README.md`** - Documentation
6. **`REMOVE-STARTUP-CONTROL.md`** - Removal instructions

### Modified Files:
1. **`robot-startup-control.service`** - Updated for port 5001
   - Changed port from 5000 to 5001
   - Updated script path to startup-control-server.py
   - Added Docker service dependency

### Unchanged Files (kept as-is):
- `web_server.py` - Robot control panel remains in main UI (port 5000)
- `requirements.txt` - Docker container requirements unchanged
- All robot code unchanged

## Architecture

### Port 5000 (Docker Container):
- Full robot web interface
- Video feed, tuning, robot control
- Only runs when robot container is active
- Uses existing web_server.py

### Port 5001 (Host Systemd Service):
- Lightweight startup control
- Always running (via systemd)
- Docker container management
- Simple Start/Stop/Shutdown interface

## Easy Removal

To completely remove this system:

```bash
cd packages/science_robot/scripts
sudo ./uninstall-host-control.sh
```

Then delete the new files listed above.

## Benefits

✅ Separate concerns (host vs container)
✅ No port conflicts
✅ Always-available startup control
✅ Minimal host dependencies
✅ Easy to remove completely
✅ Doesn't affect existing robot code

