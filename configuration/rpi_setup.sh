#bash
echo -e "\033[32m ------Configure Drone_VESPA parameters ------ \033[0m"
sudo chmod +x setup_drone_info.sh
./setup_drone_info.sh

echo -e "\033[32m ------Create service to start VESPA automatically upon booting ------ \033[0m"
chmod +x create_VESPA_service.sh
./create_VESPA_service.sh

echo -e "\033[32m ------Create VESPA shell commands ------ \033[0m"
sudo cp vespa /usr/local/bin
sudo chmod +x /usr/local/bin/vespa

echo -e "\033[32m ------Create service for Wifi provider upon booting ------ \033[0m"
original_dir=$(pwd)
cd ../src/Estimate_num_people
sudo chmod +x create_disable_wifi_service.sh
sudo chmod +x detect_wifi.sh
sudo ./create_disable_wifi_service.sh || { echo "Script execution failed"; exit 1; }
cd "$original_dir"

# Set the text color to green
echo -e "\033[32m ------ Upgrade system  ------ \033[0m"
# Update and upgrade system packages
sudo apt-get update && sudo apt-get upgrade

echo -e "\033[32m ------ Install python3 ------ \033[0m"
# Install required packages
sudo apt-get install -y python3-pip python3-dev
sudo rm /usr/lib/python3.11/EXTERNALLY-MANAGED


echo -e "\033[32m ------ Install Packages for Drone ------ \033[0m"
# Install dependencies
sudo apt-get install python3-numpy 
sudo apt install libxml2-dev libxslt-dev
pip3 install lxml

# Install Dronekit and related packages
packages=("dronekit" "dronekit-sitl" "mavproxy" "simple-pid" "pyserial")
for package in "${packages[@]}"; do
    # Check if the package is installed
    if ! pip3 show "$package" &> /dev/null; then
        # If it's not installed, install it
        pip3 install "$package"
    else
        echo "$package is already installed."
    fi
done

# Check if 'serial' is installed if yes uninstall it
if pip3 show serial &> /dev/null; then
    # If it's installed, uninstall it
    pip3 uninstall -y serial
fi


echo -e "\033[32m ------ Modify Dronekit for python3------ \033[0m"
# Modify the specified file after finding it 
sudo sed -i '2689s/collections.MutableMapping/collections.abc.MutableMapping/' "$(pip3 show dronekit | grep 'Location' | awk '{print $2}')/dronekit/__init__.py"


# Connect to mobile WiFi hotspot (Replace SSID and PASSWORD with your hotspot details)
#sudo nmcli device wifi connect SSID password PASSWORD

echo -e "\033[32m ------ Install git ------ \033[0m"
# Install git
sudo apt-get install -y git

echo -e "\033[32m ------ Set the serial comm for the drone ------ \033[0m"
# Run raspi-config in interactive mode to configure Serial UART
# Enable Serial Port hardware
# Disable login shell over serial port
# sudo raspi-config nonint do_serial_login 1

# Disable login shell over serial port in cmdline over the shell 
sudo sed -i 's/console=serial0,[0-9]* //g' /boot/cmdline.txt

# Enable Serial Port hardware by modifying /boot/config.txt
# Check if enable_uart=0 exists
if grep -q "enable_uart=0" /boot/config.txt; then
    # If exists, replace it with enable_uart=1
    sudo sed -i 's/enable_uart=0/enable_uart=1/' /boot/config.txt
elif ! grep -q "enable_uart=1" /boot/config.txt; then
    # If enable_uart=1 also doesn't exist, append it to the file
    echo "enable_uart=1" | sudo tee -a /boot/config.txt
fi

# Add the user to the dialout group
sudo usermod -a -G dialout $USER

# If the commands to disable and stop the serial-getty@ttyS0.service are not used, the serial port may still be used by the system to provide a login shell. 
# This can interfere with other uses of the serial port
# Check if serial-getty@ttyS0.service is enabled
if systemctl is-enabled --quiet serial-getty@ttyS0.service; then
    echo "serial-getty@ttyS0.service is enabled. Disabling it now..."
    sudo systemctl disable serial-getty@ttyS0.service
else
    echo "serial-getty@ttyS0.service is already disabled."
fi

# Check if serial-getty@ttyS0.service is active (running)
if systemctl is-active --quiet serial-getty@ttyS0.service; then
    echo "serial-getty@ttyS0.service is active. Stopping it now..."
    sudo systemctl stop serial-getty@ttyS0.service
else
    echo "serial-getty@ttyS0.service is already inactive."
fi

# Mask the service to prevent it from being started by any means
sudo systemctl mask serial-getty@ttyS0.service

echo -e "\033[32m ------ Disabled HDMI ------ \033[0m"
# Check if HDMI is currently enabled
if tvservice -s | grep -q "HDMI"; then
    # Disable HDMI
    sudo tvservice -o
    echo "HDMI disabled."
else
    echo "HDMI is already disabled."
fi

# Check if the /boot/config.txt file exists
if [ -e /boot/config.txt ]; then
    # Define the lines to modify/add
    declare -A lines
    lines=(
        ["dtparam=act_led_trigger"]="none"
        ["dtparam=pwr_led_trigger"]="none"
    )

    for key in "${!lines[@]}"; do
        if grep -q "^${key}" /boot/config.txt; then
            sudo sed -i "s/^${key}.*/${key}=${lines[$key]}/" /boot/config.txt
        else
            echo "${key}=${lines[$key]}" | sudo tee -a /boot/config.txt
        fi
    done

    # Ensure specific lines exist ( turn the leds off ) and 
    # Disable Bluetooth is useful if you want to use the UART without interference from the Bluetooth module, 
    # both serial and Bluetooth share the same UART resources on some Raspberry Pi models.( that ensure better serial performance)
    ensure_lines=(
        "dtparam=act_led_trigger=off"
        "dtparam=pwr_led_trigger=off"
        "dtoverlay=pi3-disable-bt"
    )

    for line in "${ensure_lines[@]}"; do
        if ! grep -q "^${line}$" /boot/config.txt; then
            echo "$line" | sudo tee -a /boot/config.txt
        fi
    done

    echo "Settings updated."
else
    echo "Error: /boot/config.txt file not found."
fi

echo -e "\033[32m ------ Install pigpio for software UART, add it as a system service ------ \033[0m"
sudo apt install pigpio
pip install pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

echo -e "\033[32m ------ Install Time of Flight LIDAR Distance software ------ \033[0m"
sudo apt-get install i2c-tools
sudo raspi-config nonint do_i2c 0   # Enable I2C 

echo -e "\033[32m ------ Install Tshark for counting people ------ \033[0m"
sudo apt-get install -y tshark
# Configure tshark to run as non-root
sudo dpkg-reconfigure wireshark-common
sudo usermod -a -G wireshark ${USER:-root}
newgrp wireshark

echo -e "\033[32m ------Configure Drone_VESPA.git ------ \033[0m"
./update_repo.sh

echo "Installation and configuration complete. Time to REBOOT"
echo -e "\033[32m *** Reboot Reconnect with ssh *** \033[0m"
sudo reboot
