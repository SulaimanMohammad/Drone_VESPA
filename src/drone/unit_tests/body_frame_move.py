from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import os
import datetime
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
        vehicle.remove_attribute_listener('velocity', on_velocity)
        vehicle.mode = VehicleMode ("LAND")
        time.sleep(3) 
        vehicle.close()
        sys.exit()

create_log_file(os.path.dirname(os.path.abspath(__file__)),  os.path.splitext(os.path.basename(__file__))[0]) 
global vehicle

#vehicle = connect (parse_connect(), wait_ready=False) # for simulation 
vehicle = connect("/dev/serial0", baud= 921600,  wait_ready=False) # for raspberry pi
#vehicle = connect("/dev/ttyUSB0", baud= 57600,  wait_ready=False, rate=10) #for telemetry 
vehicle.wait_ready(True, raise_exception=False) # for raspberry pi & telemetry only 
signal.signal(signal.SIGINT, interrupt)

drone_hight=2 
arm_and_takeoff(vehicle,drone_hight)
print( "Takeoff and wait 2 sec")

#loiter mode and hover in your place 
vehicle.mode    = VehicleMode("LOITER") #loiter mode and hover in your place 
time.sleep(1)
vehicle.mode     = VehicleMode("GUIDED")

set_data_rate(vehicle, 20)

distance=1
angl_dir= 90#-45

try:
    # angle dir in degree 
    move_body_PID(vehicle,angl_dir, distance)
except Exception as e:
    print(f"An error occurred: {str(e)}")
    vehicle.mode = VehicleMode ("LAND")
    time.sleep(2) 
    vehicle.close()

vehicle.mode    = VehicleMode("LOITER") #loiter mode and hover in your place 
time.sleep(2)
vehicle.mode     = VehicleMode("GUIDED")

# print( "move to the other direction")
# angl_dir= -180#135
# move_body_PID(vehicle,angl_dir, distance,1)


# write_log_message(f" Coming Home")
vehicle.mode = VehicleMode ("LAND")
time.sleep(2) 

# Close connection
vehicle.close()
