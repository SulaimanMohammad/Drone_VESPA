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
from expan import *
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
#vehicle = connect(parse_connect(), wait_ready=False)
# vehicle = connect("/dev/ttyUSB0", baud= 921600,  wait_ready=False, rate=10)
vehicle = connect("/dev/ttyUSB0", baud= 57600,  wait_ready=False, rate=10)
vehicle.wait_ready(True, raise_exception=False)
signal.signal(signal.SIGINT, interrupt)

set_a(3)

drone= Drone(0.0,0.0,2) # drone at the sink 
#set_home_to_zero(vehicle)
arm_and_takeoff(vehicle,drone.hight)
print( "Takeoff and wait 2 sec")

vehicle.mode    = VehicleMode("LOITER") 
time.sleep(2)
vehicle.mode     = VehicleMode("GUIDED")

hex_side=1
angl_dir=[0,120, 180,240,300,360,60,180]
#angl_dir=[0]

for i in range(len(angl_dir)):

    print( "************************* Moving to",angl_dir[i], "*************************" )    
    move_body_PID(vehicle,  drone.hight, angl_dir[i], hex_side)
    
    vehicle.mode    = VehicleMode("LOITER") 
    time.sleep(2)
    vehicle.mode     = VehicleMode("GUIDED")

time.sleep(2)


write_log_message(f" Coming Home")
vehicle.mode = VehicleMode ("LAND")
time.sleep(2) 

# Close connection
vehicle.close()
