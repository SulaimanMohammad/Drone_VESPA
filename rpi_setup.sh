#bash

# Update and upgrade system packages
sudo apt-get update && sudo apt-get upgrade

# Install required packages
sudo apt-get install -y python3-pip python3-dev
sudo apt-get install -y vim

# Install Dronekit and related packages
sudo pip3 install dronekit dronekit-sitl mavproxy

# Modify the specified file
sudo sed -i '2689s/collections.MutableMapping/collections.abc.MutableMapping/' /usr/local/lib/python3.9/dist-packages/dronekit/__init__.py

sudo pip3 install simple-pid

# Connect to mobile WiFi hotspot (Replace SSID and PASSWORD with your hotspot details)
#sudo nmcli device wifi connect SSID password PASSWORD

# Install git
sudo apt-get install -y git

# Clone the repository
sudo git clone https://github.com/SulaimanMohammad/Drone_VESPA.git

# Reboot the system
#sudo reboot

# Run raspi-config in non-interactive mode to configure Serial UART
# Enable Serial Port hardware
sudo raspi-config nonint do_serial 1
# Disable login shell over serial port
sudo raspi-config nonint do_serial_login 1

sudo sed -i 's/console=serial0,[0-9]* //g' /boot/cmdline.txt
# echo "enable_uart=1" | sudo tee -a /boot/config.txt

# Reboot the Raspberry Pi
#sudo reboot


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
    # Add the lines to disable the LEDs
    echo "dtparam=act_led_trigger=none" | sudo tee -a /boot/config.txt
    echo "dtparam=act_led_trigger=off" | sudo tee -a /boot/config.txt
    echo "dtparam=pwr_led_trigger=none" | sudo tee -a /boot/config.txt
    echo "dtparam=pwr_led_trigger=off" | sudo tee -a /boot/config.txt

    echo "LEDs disabled. Reboot your Raspberry Pi to apply the changes."
else
    echo "Error: /boot/config.txt file not found."
fi

#sudo reboot


# Uninstall serial and pyserial
pip uninstall -y serial 

# Install mavproxy and pyserial
pip install mavproxy pyserial

# Modify /boot/config.txt
# Check if enable_uart=0 exists
if grep -q "enable_uart=0" /boot/config.txt; then
    # If exists, replace it with enable_uart=1
    sudo sed -i 's/enable_uart=0/enable_uart=1/' /boot/config.txt
elif ! grep -q "enable_uart=1" /boot/config.txt; then
    # If enable_uart=1 also doesn't exist, append it to the file
    echo "enable_uart=1" | sudo tee -a /boot/config.txt
fi
sudo sed -i '$ a dtoverlay=disable-bt' /boot/config.txt

# Reboot the system
#sudo reboot

# Add the user to the dialout group
sudo usermod -a -G dialout $USER



# Clone the soft_uart repository
sudo git clone https://github.com/adrianomarto/soft_uart
cd soft_uart

# Install raspberrypi-kernel-headers package
sudo apt-get install -y raspberrypi-kernel-headers

# Compile and install the soft_uart module
sudo make
sudo make install

# Load the module with default parameters
sudo insmod soft_uart.ko

# Add the user to the dialout group

sudo usermod -a -G dialout $USER

echo "Installation and configuration complete."

# Optional: Set custom GPIO pins for TX and RX (modify as needed)
# GPIO_TX=10
# GPIO_RX=11
# sudo insmod soft_uart.ko gpio_tx=$GPIO_TX gpio_rx=$GPIO_RX
