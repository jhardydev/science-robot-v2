#!/bin/bash
# Installation script for robot startup control systemd service
# This installs the service that keeps the web interface always running

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
SERVICE_NAME="robot-startup-control.service"
SERVICE_FILE="$SCRIPT_DIR/$SERVICE_NAME"
SYSTEMD_DIR="/etc/systemd/system"

echo "Installing Science Robot Startup Control Service"
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

# Create temporary service file with actual paths
TEMP_SERVICE=$(mktemp)
sed "s|/path/to/science-robot-v2|$PROJECT_ROOT|g" "$SERVICE_FILE" > "$TEMP_SERVICE"

# Copy service file to systemd directory
echo "Copying service file to $SYSTEMD_DIR..."
cp "$TEMP_SERVICE" "$SYSTEMD_DIR/$SERVICE_NAME"

# Make sure Python script is executable
STARTUP_SCRIPT="$PROJECT_ROOT/packages/science_robot/scripts/startup-control-web.py"
if [ -f "$STARTUP_SCRIPT" ]; then
    chmod +x "$STARTUP_SCRIPT"
    echo "Made startup script executable: $STARTUP_SCRIPT"
else
    echo "Warning: Startup script not found: $STARTUP_SCRIPT"
fi

# Reload systemd
echo "Reloading systemd daemon..."
systemctl daemon-reload

echo ""
echo "Service installed successfully!"
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

rm "$TEMP_SERVICE"

