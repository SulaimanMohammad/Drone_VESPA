#bash

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
# Modify the specified file
sudo sed -i '2689s/collections.MutableMapping/collections.abc.MutableMapping/' /usr/local/lib/python3.9/dist-packages/dronekit/__init__.py


# Connect to mobile WiFi hotspot (Replace SSID and PASSWORD with your hotspot details)
#sudo nmcli device wifi connect SSID password PASSWORD

echo -e "\033[32m ------ Install git ------ \033[0m"
# Install git
sudo apt-get install -y git

echo -e "\033[32m ------ Set the serial comm for the drone ------ \033[0m"
# Run raspi-config in non-interactive mode to configure Serial UART
# Enable Serial Port hardware
sudo raspi-config nonint do_serial 1
# Disable login shell over serial port
sudo raspi-config nonint do_serial_login 1

sudo sed -i 's/console=serial0,[0-9]* //g' /boot/cmdline.txt

# Modify /boot/config.txt
# Check if enable_uart=0 exists
if grep -q "enable_uart=0" /boot/config.txt; then
    # If exists, replace it with enable_uart=1
    sudo sed -i 's/enable_uart=0/enable_uart=1/' /boot/config.txt
elif ! grep -q "enable_uart=1" /boot/config.txt; then
    # If enable_uart=1 also doesn't exist, append it to the file
    echo "enable_uart=1" | sudo tee -a /boot/config.txt
fi

# Check if the specific line exists in the file
if ! grep -q "^dtoverlay=disable-bt$" /boot/config.txt; then
    # If not, append it to the end
    echo "dtoverlay=disable-bt" | sudo tee -a /boot/config.txt
    echo "dtoverlay=disable-bt added."
else
    echo "dtoverlay=disable-bt is already present in /boot/config.txt."
fi

# Add the user to the dialout group
sudo usermod -a -G dialout $USER

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
    # Define the lines to check and modify/add
    declare -A lines
    lines=(["dtparam=act_led_trigger="]="none"
           ["dtparam=pwr_led_trigger="]="none"
    )

    # Loop through the lines to modify or add them if they don't exist
    for key in "${!lines[@]}"; do
        if grep -q "${key}.*" /boot/config.txt; then
            # If the line exists, modify it
            sudo sed -i "s/${key}.*/${key}${lines[$key]}/" /boot/config.txt
        else
            # If the line doesn't exist, append it to the file
            echo "${key}${lines[$key]}" | sudo tee -a /boot/config.txt
        fi
    done
    
    # Lines to just ensure exist
    ensure_lines=("dtparam=act_led_trigger=off"
                  "dtparam=pwr_led_trigger=off"
                  "dtoverlay=disable-bt"
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

echo -e "\033[32m ------Configure Drone_VESPA.git ------ \033[0m"
./update_repo.sh

echo "Installation and configuration complete. Time to REBOOT"
echo -e "\033[32m *** Reboot Reconnect with ssh *** \033[0m"
sudo reboot
