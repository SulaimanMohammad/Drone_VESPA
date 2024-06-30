#!/bin/bash

# Find the username of the Raspberry Pi
USERNAME=$(whoami)

# Define the home directory and the script path
HOME_DIR="/home/$USERNAME"
SCRIPT_PATH="$HOME_DIR/Drone_VESPA/main.py"

# Create the systemd service file
SERVICE_FILE="/etc/systemd/system/VESPA.service"

# Create the content of the service file
SERVICE_CONTENT="[Unit]
Description=VESPA Python Script
After=network.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 -u $SCRIPT_PATH # -u flag, which forces the stdout and stderr streams to be unbuffered 
WorkingDirectory=$HOME_DIR
StandardOutput=journal
StandardError=journal
Restart=always
User=$USERNAME

[Install]
WantedBy=multi-user.target
"

# Write the service content to the service file
echo "$SERVICE_CONTENT" | sudo tee $SERVICE_FILE > /dev/null

# Reload the systemd daemon
sudo systemctl daemon-reload

# Enable the service to start at boot
sudo systemctl enable VESPA.service

# Start the service immediately
sudo systemctl start VESPA.service