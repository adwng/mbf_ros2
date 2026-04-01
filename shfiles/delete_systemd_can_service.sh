#!/bin/bash
# This script stops, disables, and deletes the can.service systemd service.
# Run this script as root or with sudo.

SERVICE_FILE="/etc/systemd/system/can.service"

# Ensure the script is run as root
if [ "$(id -u)" -ne 0 ]; then
    echo "Please run this script as root (e.g., using sudo)."
    exit 1
fi

# Check if the service file exists
if [ -f "$SERVICE_FILE" ]; then
    # Stop the service if it's running
    systemctl stop can.service
    # Disable the service to prevent it from starting on boot
    systemctl disable can.service
    # Remove the service file
    rm "$SERVICE_FILE"
    # Reload systemd to apply changes
    systemctl daemon-reload
    echo "CAN service removed successfully."
else
    echo "Error: can.service not found at $SERVICE_FILE"
    exit 1
fi
