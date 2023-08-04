
import os
import datetime
import sys
import signal
# Get the parent directory path
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
from expan import *
from drone_ardupilot import *


def interrupt(signal_num, frame):
    print("Interrupted!")
    global vehicle
    if vehicle is not None:
        vehicle.mode = VehicleMode ("LAND")
        time.sleep(3) 
        vehicle.close()
        sys.exit()
        

create_log_file(os.path.dirname(os.path.abspath(__file__)),  os.path.splitext(os.path.basename(__file__))[0]) 
global vehicle
# vehicle = connect(parse_connect(), wait_ready=False)
vehicle = connect("/dev/ttyUSB0", wait_ready=False,baud=57600)
vehicle.wait_ready(True, raise_exception=False)
signal.signal(signal.SIGINT, interrupt)

print(" Rebooting ......")
vehicle.reboot()
time.sleep(1)

vehicle.close()
