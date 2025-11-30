#!/bin/bash
# Installation script for robot startup control systemd service (host-level)
# This installs the startup control web server that runs on port 5001
#
# TO REMOVE/UNDO:
#   1. sudo systemctl stop robot-startup-control.service
#   2. sudo systemctl disable robot-startup-control.service
#   3. sudo rm /etc/systemd/system/robot-startup-control.service
#   4. sudo systemctl daemon-reload
#   5. pip3 uninstall flask psutil (if not needed elsewhere)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
SERVICE_NAME="robot-startup-control.service"
SERVICE_FILE="$SCRIPT_DIR/$SERVICE_NAME"
SYSTEMD_DIR="/etc/systemd/system"
REQUIREMENTS_FILE="$SCRIPT_DIR/requirements-host.txt"

echo "========================================="
echo "Robot Startup Control Installation"
echo "========================================="
echo "Project root: $PROJECT_ROOT"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Error: This script must be run as root (use sudo)"
    exit 1
fi

# Check if service file exists
if [ ! -f "$SERVICE_FILE" ]; then
    echo "Error: Service file not found: $SERVICE_FILE"
    exit 1
fi

# Check if startup control server script exists
STARTUP_SCRIPT="$SCRIPT_DIR/startup-control-server.py"
if [ ! -f "$STARTUP_SCRIPT" ]; then
    echo "Error: Startup control server script not found: $STARTUP_SCRIPT"
    exit 1
fi

# Install Python dependencies (Flask, psutil)
echo "Step 1: Installing Python dependencies..."
if [ -f "$REQUIREMENTS_FILE" ]; then
    echo "Installing from $REQUIREMENTS_FILE"
    pip3 install --user -r "$REQUIREMENTS_FILE" || {
        echo "Warning: pip3 install failed, trying with sudo..."
        pip3 install -r "$REQUIREMENTS_FILE" || {
            echo "Error: Failed to install Python dependencies"
            echo "Please install manually: pip3 install flask psutil"
            exit 1
        }
    }
else
    echo "Warning: requirements-host.txt not found, installing Flask and psutil directly"
    pip3 install --user flask psutil || pip3 install flask psutil
fi

# Make startup script executable
echo ""
echo "Step 2: Making startup script executable..."
chmod +x "$STARTUP_SCRIPT"
echo "✓ Startup script is executable"

# Create temporary service file with actual paths
echo ""
echo "Step 3: Creating systemd service file..."
TEMP_SERVICE=$(mktemp)
sed "s|/path/to/science-robot-v2|$PROJECT_ROOT|g" "$SERVICE_FILE" > "$TEMP_SERVICE"

# Copy service file to systemd directory
cp "$TEMP_SERVICE" "$SYSTEMD_DIR/$SERVICE_NAME"
echo "✓ Service file installed to $SYSTEMD_DIR/$SERVICE_NAME"

# Reload systemd
echo ""
echo "Step 4: Reloading systemd daemon..."
systemctl daemon-reload
echo "✓ Systemd daemon reloaded"

# Clean up temp file
rm "$TEMP_SERVICE"

echo ""
echo "========================================="
echo "Installation completed successfully!"
echo "========================================="
echo ""
echo "To start the service:"
echo "  sudo systemctl start $SERVICE_NAME"
echo ""
echo "To enable auto-start on boot:"
echo "  sudo systemctl enable $SERVICE_NAME"
echo ""
echo "To check service status:"
echo "  sudo systemctl status $SERVICE_NAME"
echo ""
echo "To view logs:"
echo "  sudo journalctl -u $SERVICE_NAME -f"
echo ""
echo "Access the control panel at:"
echo "  http://robot1.local:5001"
echo "  or http://<robot-ip>:5001"
echo ""
echo "========================================="
echo "TO REMOVE/UNDO:"
echo "  1. sudo systemctl stop $SERVICE_NAME"
echo "  2. sudo systemctl disable $SERVICE_NAME"
echo "  3. sudo rm $SYSTEMD_DIR/$SERVICE_NAME"
echo "  4. sudo systemctl daemon-reload"
echo "  5. Delete files:"
echo "     - $STARTUP_SCRIPT"
echo "     - $REQUIREMENTS_FILE"
echo "     - $SERVICE_FILE"
echo "     - $SCRIPT_DIR/install-host-control.sh"
echo "========================================="

