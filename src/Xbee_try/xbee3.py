import sys
import os
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
from Xbee_module.xbee_usb import connect_xbee,close_xbee_port
from VESPA.VESPA_module import *
from VESPA.expansion import expansion_listener,expand_and_form_border_try
from VESPA.spanning import spanning
# Serial port settings for XBee
serial_port = '/dev/ttyUSB2'  # Default UART port on Raspberry Pi
baud_rate = 9600  # Baud rate for XBee
connect_xbee(serial_port, baud_rate) 
sq3=1.73
a=13
x=(sq3 / 2.0) * a
y=(3.0 / 2.0) * a
drone = Drone(3,0.0,0.0,1)
time.sleep(3)       
try:
    while True:
        expand_and_form_border_try(drone)
        spanning(drone)
        time.sleep(100)  # Interval for sending messages
except KeyboardInterrupt:
    print("Interrupt received, stopping...")
    running = False  # Signal the reading thread to stop
finally:
    # Close the serial connection
    close_xbee_port()
    print("Serial connection closed.")