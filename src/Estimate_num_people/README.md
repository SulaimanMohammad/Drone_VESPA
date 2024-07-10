# WiFi-Based People Estimation
This repository provides a method for estimating the number of people in a given area by detecting nearby phones through their WiFi signals. The system leverages Tshark and a wireless USB adapter in monitoring mode, connected to a Raspberry Pi.

## Features
- **Detect WiFi Signals:** Identifies and counts the number of phones by their WiFi signals.
- **Automated Setup:** Configures the necessary environment and dependencies.
- **Data Collection:** Gathers and processes WiFi signal data to estimate the number of people.

## Requirements
- Raspberry Pi
- Wireless USB Adapter capable of monitoring mode
- Internet connection for initial setup

## Installation and running
1. Clone this repository to your Raspberry Pi.
   ```bash
      git clone https://github.com/SulaimanMohammad/wifi_counter.git
      cd wifi_counter
   ```
2. Run the detect_wifi bash script to set up and start the detection process.
   ```bash
      ./detect_wifi.sh <scan_max_time> <RSSI>
   ```
   - scan_max_time: time of scan spent on each channel in second
   - RSSI: Received Signal Strength Indicator, is an indication of the power level being received by the receiving radio after the antenna and possible cable loss. Therefore, the greater the RSSI value, the stronger the signal. Thus, when an RSSI value is represented in a negative form (e.g. −100), the closer the value is to 0, the stronger the received signal has been.

## detect_wifi Script
This script automates the setup and execution of the WiFi-based people estimation process. It performs the following steps:

1. **Download OUI Table**: Retrieves the OUI table containing MAC addresses of various hardware manufacturers.
2. **Filter Non-Phone MACs**: Extracts and saves MAC addresses of non-phone devices to `Non_phones_macs` file.
3. **Install Tshark**: Checks if Tshark is installed and installs it if necessary.
4. **Configure Wireless Adapter**: Sets the wireless adapter to monitoring mode.
5. **Launch Tshark**: Starts Tshark to scan for WiFi beacons and probes.
6. **Analyes data**: Call a pythonn script to read `unique.txt` and retuen the number of phones found

## Data Collection Process
- **Monitoring**: Tshark scans for WiFi *beacons and probes* from devices searching for or connected to WiFi networks.
- **Channel Hopping**: The script changes channels to ensure all devices are detected:
   - Main Channels (1, 6, 11): Most phones work on those. Scans for **scan_max_time** seconds.
   - Secondary Channels (2, 3, 4, 5, 7, 8, 9, 10): Scans for scan_max_time / 4 seconds due to minimal phone presence.
- **AProximity Devices:**: Uses RSSI Uses RSSI (signal strength) to define the range of devices to capture. Tshark filters signals weaker than the specified **RSSI**.
- **Data Storage**: All collected information is saved in `data.txt` (contains raw data collected from Tshark).
- **Data Filtering**: Filters the data to save only unique MAC addresses in `unique.txt`.
- **Analyes data**: Extracts MAC addresses in `unique.txt` to identify phone devices.

## Note:
- Ensure your wireless adapter supports monitoring mode.
- The script requires root privileges to set the adapter in monitoring mode and to run Tshark.

# How Phones are Detected
Tshark collects all beacons and probes from devices within the range defined by RSSI across all channels. The results are saved as MAC addresses in `data.txt`.

## Random MAC
Recent phones often use random MAC addresses that change over time, making traditional OUI lookup tables ineffective. Therefore, the script uses sequence numbers, representing connection attempts or data transmissions over WiFi, to filter and identify unique devices.

`detect_wifi.sh` filters MACs to keep only the unique ones representing a single device, regardless of sequence numbers since the same MAC is repeated.

If a device use random MAC addresses:
```bash
70:DA:17:04:C1:3E 1050
70:DA:17:04:C1:3B 1052
70:DA:17:04:C1:4A 1056
70:DA:17:04:C1:9C 1053
```
These addresses represent the same device. The script considers the prefix (first 6 digits of the MAC) as one device.
`analyes_data.py`: processes all MACs in `unique.txt` with consideration of sequence number to generate a list of prefixes represent all the avilable unique devices around.

## Phone Identification
When phones have random MACs, the prefix will not match any in `Non_phones_macs`. Other devices like laptops and routers will have fixed MAC addresses listed in `Non_phones_macs`.
- If a prefix is found, it is not a phone it is another device with fixed prefixe for the manufacturer.
- If a prefix is not found, it is either a phone's manufacturer prefix ( not included in `Non_phones_macs`) or a random prefix representing a phone.


