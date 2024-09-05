#!/bin/bash
INTERFACE="wlan1"
FLAG_FILE="/tmp/disable_wifi"

# Check if the interface is up
if ifconfig $INTERFACE | grep -q "UP"; then
    # Bring down WiFi interface if it's up
    ifconfig $INTERFACE down
fi

# Check if the flag file exists, and delete it if it does
if [ -f $FLAG_FILE ]; then
    rm -f $FLAG_FILE
fi