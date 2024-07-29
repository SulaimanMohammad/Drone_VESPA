#!/bin/bash

#Create service to be launched with booting with stop wifi adapter until use
# Variables
SERVICE_NAME_DELAYED="disable_wifi_delayed.service"
SERVICE_NAME_NORMAL="disable_wifi.service"
SERVICE_PATH_DELAYED="/etc/systemd/system/$SERVICE_NAME_DELAYED"
SERVICE_PATH_NORMAL="/etc/systemd/system/$SERVICE_NAME_NORMAL"
SCRIPT_PATH="/usr/local/bin/disable_wifi.sh"
INTERFACE="wlan1"
FLAG_FILE="/tmp/disable_wifi"

# Check if the script is run with sudo
if [ "$EUID" -ne 0 ]; then
  echo "Please run this script as root or with sudo"
  exit 1
fi

# Create the delayed boot Systemd service file
cat <<EOF > $SERVICE_PATH_DELAYED
[Unit]
Description=Disable WiFi on Boot with Delay
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=$SCRIPT_PATH
ExecStartPost=/bin/sleep 10

[Install]
WantedBy=multi-user.target
EOF

# Create the normal operation Systemd service file
cat <<EOF > $SERVICE_PATH_NORMAL
[Unit]
Description=Disable WiFi
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=$SCRIPT_PATH

[Install]
WantedBy=multi-user.target
EOF

# Create the disable_wifi.sh script
cat <<EOF > $SCRIPT_PATH
#!/bin/bash

if [ ! -f $FLAG_FILE ]; then
  /sbin/ifconfig $INTERFACE down
fi
EOF

# Set the appropriate permissions for the service files and script
chmod 644 $SERVICE_PATH_DELAYED
chmod 644 $SERVICE_PATH_NORMAL
chmod +x $SCRIPT_PATH

# Reload systemd to recognize the new services
systemctl daemon-reload

# Enable and start the delayed boot service
systemctl enable $SERVICE_NAME_DELAYED

# Enable the normal operation service
systemctl enable $SERVICE_NAME_NORMAL

# Optionally start the normal operation service immediately
systemctl start $SERVICE_NAME_NORMAL

echo "Systemd services $SERVICE_NAME_DELAYED and $SERVICE_NAME_NORMAL created, enabled, and started."


#----------------------------------------------------
# Give premission to control wifi without passward
#----------------------------------------------------


USERNAME=$(whoami)
COMBINED_SCRIPT="/home/drone1/Drone_VESPA/src/Estimate_num_people/scan_channels.sh"
SUDOERS_TMP="/tmp/sudoers.tmp"

# Backup the current sudoers file
cp /etc/sudoers /etc/sudoers.bak

# Add the specific command permissions to a temporary sudoers file
cat /etc/sudoers > $SUDOERS_TMP
echo "$USERNAME ALL=(ALL) NOPASSWD: $COMBINED_SCRIPT" >> $SUDOERS_TMP

# Validate the temporary sudoers file
visudo -cf $SUDOERS_TMP
if [ $? -eq 0 ]; then
  # If the validation is successful, replace the sudoers file
  cp $SUDOERS_TMP /etc/sudoers
  echo "Sudoers file updated successfully."
else
  # If the validation fails, restore the backup
  cp /etc/sudoers.bak /etc/sudoers
  echo "Failed to validate the sudoers file. Changes have been reverted."
fi

# Clean up
rm $SUDOERS_TMP



