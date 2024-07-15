#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 2 ]; then
  echo "Usage: $0 <SCAN_TIME> <RSSI_THRESHOLD>"
  exit 1
fi

# Get time of research
PRIMARY_SCAN_TIME=$1  # For the main channels 1,6,11
SECONDARY_SCAN_TIME=$(( SCAN_TIME / 4 ))  # For the rest of channels

# Get the RSSI threshold from the first argument
RSSI_THRESHOLD=$2

# Change to the directory where the script is located (save all fils in this dir)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Check if OUI_table.txt and Non_phones_macs.txt do not exist
if [ ! -f "OUI_table.txt" ] && [ ! -f "Non_phones_macs.txt" ]; then
    # Download the file and save it as OUI_table.txt
    wget "https://devtools360.com/en/macaddress/vendorMacs.xml?download=true" --output-document=OUI_table.txt > /dev/null 2>&1

    # Filter out specific manufacturers and extract MAC prefixes
    grep -vEi 'Motorola Mobility LLC, a Lenovo Company|GUANGDONG OPPO MOBILE TELECOMMUNICATIONS CORP.,LTD|Huawei Symantec Technologies Co.,Ltd.|Microsoft|HTC Corporation|Samsung Electronics Co.,Ltd|SAMSUNG ELECTRO-MECHANICS(THAILAND)|BlackBerry RTS|LG ELECTRONICS INC|Apple, Inc.|LG Electronics|OnePlus Tech (Shenzhen) Ltd|Xiaomi Communications Co Ltd|LG Electronics \(Mobile Communications\)' OUI_table.txt | sed -E 's/.*mac_prefix="([^"]*)".*/\1/' > Non_phones_macs.txt
fi

if ! command -v tshark &> /dev/null; then
    # Install tshark
    sudo apt-get update
    sudo apt-get install -y tshark

    # Configure tshark to run as non-root
    sudo dpkg-reconfigure wireshark-common
    sudo usermod -a -G wireshark ${USER:-root}
    newgrp wireshark
fi

# Check if wlan1 is in monitor mode
if ! iw wlan1 info | grep -q "type monitor"; then
    sudo ifconfig wlan1 down
    sudo iw wlan1 set monitor none
    sudo ifconfig wlan1 up
fi

filename="data.txt"

# Check if the file exists
if [ -f "$filename" ]; then
    # File exists, delete it
    rm "$filename"
    rm unique.txt
fi

# Create a new empty file
touch "$filename"

# Specify the channels to iterate over
PRIMARY_CHANNELS=(1 6 11) # 1 ,6,11 are the ones used by smartphones
SECONDARY_CHANNELS=(2 3 4 5 7 8 9 10)

# Run tshark on primary channels for the full scan time
for channel in "${PRIMARY_CHANNELS[@]}"; do
    # Capture packets on the current channel and append to data.txt
    sudo iw dev wlan1 set channel $channel
    tshark -i wlan1  -a duration:$PRIMARY_SCAN_TIME -T fields -e wlan.sa -e wlan.seq -e radiotap.dbm_antsignal 2>/dev/null | \
    awk -v threshold="$RSSI_THRESHOLD" '{
        if ($3 ~ /,/) {
        split($3, a, ",");
        avg = (a[1] + a[2]) / 2;
        if (avg > threshold)
            print $1, $2
        }
    }' >> data.txt
done


# Run tshark on secondary channels for the calculated scan time
for channel in "${SECONDARY_CHANNELS[@]}"; do
    sudo iw dev wlan1 set channel $channel
    tshark -i wlan1 -a duration:$SECONDARY_SCAN_TIME -T fields -e wlan.sa -e wlan.seq -e radiotap.dbm_antsignal 2>/dev/null | \
    awk -v threshold="$RSSI_THRESHOLD" '{
        if ($3 ~ /,/) {
        split($3, a, ",");
        avg = (a[1] + a[2]) / 2;
        if (avg > threshold)
            print $1, $2
        }
    }' >> data.txt
done

awk '{if (!seen[$1] || $2 > seen[$1]) seen[$1] = $2} END {for (mac in seen) print mac, seen[mac]}' data.txt > unique.txt

# Call the Python script and capture its output
number_of_phones=$(python analyse_data.py)
