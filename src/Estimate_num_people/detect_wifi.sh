#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <SCAN_TIME> <RSSI_THRESHOLD>"
  exit 1
fi

SCAN_TIME=$1
RSSI_THRESHOLD=$2
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMBINED_SCRIPT="$SCRIPT_DIR/scan_channels.sh"
ANALYSE_DATA_SCRIPT="$SCRIPT_DIR/analyse_data.py"

# Call the scan scan_channels control script with sudo
sudo bash "$COMBINED_SCRIPT" "$SCAN_TIME" "$RSSI_THRESHOLD"

# Call the Python script and capture its output
number_of_phones=$(python3 "$ANALYSE_DATA_SCRIPT")

echo $number_of_phones