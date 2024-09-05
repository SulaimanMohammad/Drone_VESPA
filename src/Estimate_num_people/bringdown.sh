#!/bin/bash
INTERFACE="wlan1"
FLAG_FILE="/tmp/disable_wifi"

# Check if the interface is up
if ifconfig $INTERFACE | grep -q "UP"; then
    # Bring down WiFi interface if it's up
    ifconfig $INTERFACE down
    echo "$INTERFACE is down"
else
    echo "$INTERFACE is already down"
fi

# Check if the flag file exists, and delete it if it does
if [ -f $FLAG_FILE ]; then
    rm -f $FLAG_FILE
    echo "Flag file $FLAG_FILE deleted"
else
    echo "Flag file $FLAG_FILE does not exist"
fi
