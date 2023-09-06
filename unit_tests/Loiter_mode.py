
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
import math 

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
        

create_log_file(os.path.dirname(os.path.abspath(__file__)),  os.path.splitext(os.path.basename(__file__))[0]) 
global vehicle
# vehicle = connect (connection_string, wait_ready=False) # for simulation 
vehicle = connect("/dev/serial0", baud= 921600,  wait_ready=False,rate=10) # for raspberry p
#vehicle = connect("/dev/ttyUSB0", baud= 57600,  wait_ready=False, rate=10) #for telemetry 
vehicle.wait_ready(True, raise_exception=False)

signal.signal(signal.SIGINT, interrupt)
vehicle.mode    = VehicleMode("STABILIZE")

arm_and_takeoff(vehicle,2)
# time.sleep(2)

# write_log_message(f" current_altitude= {vehicle.location.global_relative_frame.alt}")
vehicle.mode    = VehicleMode("LOITER") #loiter mode and hover in your place 
# write_log_message(f" current_altitude= {vehicle.location.global_relative_frame.alt}")
time.sleep(5)

write_log_message(f" LAND")
vehicle.mode = VehicleMode ("LAND")
time.sleep(3) 

vehicle.close()
