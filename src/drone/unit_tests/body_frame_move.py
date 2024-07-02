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
    go_to_ref_altitude(vehicle)
    time.sleep(2)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.mode = VehicleMode ("LAND")
    time.sleep(2) 
    # Close connection
    vehicle.close()

except Exception as e:
    print(f"An error occurred: {str(e)}")
    vehicle.mode = VehicleMode ("LAND")
    time.sleep(2) 
    vehicle.close()

