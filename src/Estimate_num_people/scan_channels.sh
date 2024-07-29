#!/bin/bash

# Variables
INTERFACE="wlan1"
PRIMARY_CHANNELS=(1 6 11)
SECONDARY_CHANNELS=(2 3 4 5 7 8 9 10)
PRIMARY_SCAN_TIME=$1
SECONDARY_SCAN_TIME=$(( PRIMARY_SCAN_TIME / 4 ))
RSSI_THRESHOLD=$2
FLAG_FILE="/tmp/disable_wifi"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FILENAME="$SCRIPT_DIR/data.txt"
FILENAME_UNIQUE="$SCRIPT_DIR/unique.txt"


# Check if OUI_table.txt and Non_phones_macs.txt do not exist
if [ ! -f "OUI_table.txt" ] && [ ! -f "Non_phones_macs.txt" ]; then
    # Down*load the file and save it as OUI_table.txt
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


# Bring up WiFi interface
ifconfig $INTERFACE up
touch $FLAG_FILE

# Set monitor mode if not already set
if ! iw $INTERFACE info | grep -q "type monitor"; then
    ifconfig $INTERFACE down
    iw $INTERFACE set monitor none
    ifconfig $INTERFACE up
fi

# Remove old data files if they exist
rm -f "$FILENAME" "$FILENAME_UNIQUE"

# Create new empty data files
touch "$FILENAME"

# Function to scan a given channel
scan_channel() {
    local channel=$1
    local duration=$2
    iw dev $INTERFACE set channel $channel
    tshark -i $INTERFACE -a duration:$duration -T fields -e wlan.sa -e wlan.seq -e radiotap.dbm_antsignal 2>/dev/null | \
    awk -v threshold="$RSSI_THRESHOLD" '{
        if ($3 ~ /,/) {
        split($3, a, ",");
        avg = (a[1] + a[2]) / 2;
        if (avg > threshold)
            print $1, $2
        }
    }' >> "$FILENAME"
}

# Scan primary channels
for channel in "${PRIMARY_CHANNELS[@]}"; do
    scan_channel $channel $PRIMARY_SCAN_TIME
done

# Scan secondary channels
for channel in "${SECONDARY_CHANNELS[@]}"; do
    scan_channel $channel $SECONDARY_SCAN_TIME
done

# Process captured data
awk '{if (!seen[$1] || $2 > seen[$1]) seen[$1] = $2} END {for (mac in seen) print mac, seen[mac]}' "$FILENAME" > "$FILENAME_UNIQUE"

# Bring down WiFi interface
ifconfig $INTERFACE down
rm -f $FLAG_FILE

