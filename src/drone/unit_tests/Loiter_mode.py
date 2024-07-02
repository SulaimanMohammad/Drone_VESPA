import os
import sys
import signal

# Get the parent directory path
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
from drone_ardupilot import *

#set_a(20)

create_log_file(os.path.dirname(os.path.abspath(__file__)),  os.path.splitext(os.path.basename(__file__))[0]) 

def interrupt(signal_num, frame):
    print("Interrupted!")
    global vehicle
    if vehicle is not None:
        vehicle.mode = VehicleMode ("LAND")
        time.sleep(3) 
        vehicle.close()
        sys.exit()
        

create_log_file()
global vehicle
vehicle=drone_connect()
signal.signal(signal.SIGINT, interrupt)
vehicle.mode    = VehicleMode("STABILIZE")

drone_hight=2 
arm_and_takeoff(vehicle,drone_hight)

# write_log_message(f" current_altitude= {vehicle.location.global_relative_frame.alt}")
vehicle.mode    = VehicleMode("LOITER") #loiter mode and hover in your place 
# write_log_message(f" current_altitude= {vehicle.location.global_relative_frame.alt}")
time.sleep(5)

write_log_message(f" LAND")
vehicle.mode = VehicleMode ("LAND")
time.sleep(3) 

vehicle.close()
