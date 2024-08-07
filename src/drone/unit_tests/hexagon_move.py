from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
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
        vehicle.remove_attribute_listener('velocity', on_velocity)
        vehicle.mode = VehicleMode ("LAND")
        time.sleep(3) 
        vehicle.close()
        sys.exit()

create_log_file() 
global vehicle
vehicle=drone_connect()
signal.signal(signal.SIGINT, interrupt)

drone_hight=2 
arm_and_takeoff(vehicle,drone_hight)

print( "Takeoff and wait 2 sec")

wait_and_hover(vehicle, 2)

hex_side=1
angl_dir=[0,120, 180,240,300,360,60,180]
#angl_dir=[0]

for i in range(len(angl_dir)):

    print( "************************* Moving to",angl_dir[i], "*************************" )    
    move_body_PID(vehicle, angl_dir[i], hex_side)
    
    wait_and_hover(vehicle, 2)

time.sleep(2)


write_log_message(f" Coming Home")
vehicle.mode = VehicleMode ("LAND")
time.sleep(2) 

# Close connection
vehicle.close()
