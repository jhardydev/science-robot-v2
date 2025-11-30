#!/bin/bash
# Uninstall script for robot startup control systemd service
# This completely removes the host-level startup control server (port 5001)
#
# After running this, you can easily reinstall by running install-host-control.sh again

set -e

SERVICE_NAME="robot-startup-control.service"
SYSTEMD_DIR="/etc/systemd/system"

echo "========================================="
echo "Removing Robot Startup Control Server"
echo "========================================="
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Error: This script must be run as root (use sudo)"
    exit 1
fi

# Stop and disable service
if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
    echo "Step 1: Stopping service..."
    systemctl stop "$SERVICE_NAME"
    echo "✓ Service stopped"
fi

if systemctl is-enabled --quiet "$SERVICE_NAME" 2>/dev/null; then
    echo "Step 2: Disabling service..."
    systemctl disable "$SERVICE_NAME"
    echo "✓ Service disabled"
fi

# Remove service file
SERVICE_FILE="$SYSTEMD_DIR/$SERVICE_NAME"
if [ -f "$SERVICE_FILE" ]; then
    echo "Step 3: Removing service file..."
    rm "$SERVICE_FILE"
    echo "✓ Service file removed"
else
    echo "Step 3: Service file not found (already removed?)"
fi

# Reload systemd
echo "Step 4: Reloading systemd..."
systemctl daemon-reload
echo "✓ Systemd reloaded"

echo ""
echo "========================================="
echo "Service removed successfully!"
echo "========================================="
echo ""
echo "Optional: Remove Python dependencies (only if not needed elsewhere):"
echo "  pip3 uninstall flask psutil"
echo ""
echo "Optional: Delete installation files from packages/science_robot/scripts/:"
echo "  - startup-control-server.py"
echo "  - requirements-host.txt"
echo "  - install-host-control.sh"
echo "  - uninstall-host-control.sh"
echo "  - REMOVE-STARTUP-CONTROL.md"
echo ""
echo "To reinstall later, run: sudo ./install-host-control.sh"
echo "========================================="

