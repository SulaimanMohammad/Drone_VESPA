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


create_log_file() 
global vehicle

vehicle=drone_connect()
signal.signal(signal.SIGINT, interrupt)

drone_hight=2 
arm_and_takeoff(vehicle,drone_hight)
print( "Takeoff and wait 2 sec")
time.sleep(2)

defined_groundspeed=1
distance=10
angl_dir= 90

current_lat = vehicle.location.global_relative_frame.lat
current_lon = vehicle.location.global_relative_frame.lon
# angle dir in degree 
long, lat= new_coordinates(current_lon, current_lat, distance, angl_dir)
point1 = LocationGlobalRelative( lat,long,drone_hight)
vehicle.simple_goto( point1, groundspeed=defined_groundspeed)
# simple_goto will retuen after the command is sent, thus you need to sleep to give the drone time to move 
time.sleep( (distance/defined_groundspeed)+1 )
#loiter mode and hover in your place if it is before sleep then the drone will not move
vehicle.mode    = VehicleMode("LOITER") #loiter mode and hover in your place 
time.sleep(2)
vehicle.mode     = VehicleMode("GUIDED")

print( "move to the other direction")
angl_dir= 270#135
# angle dir in degree new_coordinates(vehicle, distance, angl_dir)
long, lat= new_coordinates(current_lon, current_lat, distance, angl_dir)
point2 = LocationGlobalRelative( lat,long,drone_hight)
vehicle.simple_goto( point2, groundspeed=defined_groundspeed)

time.sleep( (distance/defined_groundspeed)+1 ) 
write_log_message(f" Coming Home")
print( "Coming Home")
vehicle.mode = VehicleMode ("LAND")
# Close connection
vehicle.close()
