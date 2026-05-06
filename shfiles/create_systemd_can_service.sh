#!/bin/bash
# This script copies the can.service file (located in the same folder)
# to /etc/systemd/system/, then reloads systemd, enables, and starts the service.
#
# Run this script as root or with sudo.

# Ensure the script is run as root
if [ "$(id -u)" -ne 0 ]; then
    echo "Please run this script as root (e.g., using sudo)."
    exit 1
fi

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SERVICE_SRC="$SCRIPT_DIR/can.service"
SERVICE_DEST="/etc/systemd/system/can.service"

# Check if can.service exists in the script directory
if [ ! -f "$SERVICE_SRC" ]; then
    echo "Error: can.service not found in $SCRIPT_DIR"
    exit 1
fi

# Copy the service file to /etc/systemd/system
cp "$SERVICE_SRC" "$SERVICE_DEST"
chmod 644 "$SERVICE_DEST"

# Reload systemd to pick up the new service file
systemctl daemon-reload

# Enable the service to run at boot and start it now
systemctl enable can.service
systemctl start can.service

echo "CAN service installed, enabled, and started successfully."