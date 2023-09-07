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

# def on_velocity(self, attribute_name, value):
#     print("Velocity: %s" % str(value))

create_log_file(os.path.dirname(os.path.abspath(__file__)),  os.path.splitext(os.path.basename(__file__))[0]) 
global vehicle

# vehicle = connect (parse_connect(), wait_ready=False) # for simulation 
vehicle = connect("/dev/serial0", baud= 921600,  wait_ready=False,rate=10) # for raspberry p
#vehicle = connect("/dev/ttyUSB0", baud= 57600,  wait_ready=False, rate=10) #for telemetry
vehicle.wait_ready(True, raise_exception=False) # for raspberry pi & telemetry only 
signal.signal(signal.SIGINT, interrupt)

set_a(3)
# vehicle.add_attribute_listener('velocity', on_velocity)

drone= Drone(0.0,0.0,2) # drone at the sink 
#set_home_to_zero(vehicle)
arm_and_takeoff(vehicle,drone.hight)
print( "Takeoff and wait 2 sec")
time.sleep(2)

vehicle.mode    = VehicleMode("LOITER") #loiter mode and hover in your place 
time.sleep(2)
vehicle.mode     = VehicleMode("GUIDED")

distance=100
angl_dir= -45

# angle dir in degree 
lat,long= new_coordinates(vehicle, distance, angl_dir)
point1 = LocationGlobalRelative( lat,long,drone.hight)
vehicle.simple_goto( point1, groundspeed=1)

# vehicle.mode    = VehicleMode("LOITER") #loiter mode and hover in your place 
vehicle.mode     = VehicleMode("GUIDED")

print( "move to the other direction")
time.sleep(5)

angl_dir= 45#135

# angle dir in degree new_coordinates(vehicle, distance, angl_dir)
lat,long= new_coordinates(vehicle, distance, angl_dir)
point1 = LocationGlobalRelative( lat,long,drone.hight)
vehicle.simple_goto( point1, groundspeed=1)

write_log_message(f" Coming Home")

print( "Coming Home")
vehicle.mode = VehicleMode ("LAND")
time.sleep(2) 

# Close connection
vehicle.close()
