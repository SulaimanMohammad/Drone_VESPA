
import os
import sys
import signal
# Get the parent directory path
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
from drone_ardupilot import *


def interrupt(signal_num, frame):
    print("Interrupted!")
    global vehicle
    if vehicle is not None:
        vehicle.mode = VehicleMode ("LAND")
        time.sleep(3) 
        vehicle.close()
        sys.exit()
        
global vehicle
vehicle=drone_connect()
signal.signal(signal.SIGINT, interrupt)

print(" Rebooting ......")
vehicle.reboot()
time.sleep(1)

vehicle.close()
