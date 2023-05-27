
import os
import datetime
import sys
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command

# Get the parent directory path
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
from expan import *
from drone_ardupilot import *
import math 

#set_a(20)

create_log_file(os.path.dirname(os.path.abspath(__file__)),  os.path.splitext(os.path.basename(__file__))[0]) 

vehicle = connect(parse_connect(), wait_ready=False)
#set_home_to_zero(vehicle)
# vehicle.mode    = VehicleMode("AUTO")
# vehicle.mode    = VehicleMode("CIRCLE")
#vehicle.parameters['CIRCLE_OPTIONS']=1 # face the direction of the travel 
#vehicle.parameters['CIRCLE_RADIUS']=2000 # set the parameter of the radius of the circle 
arm_and_takeoff(vehicle, 10)

time.sleep(1)
alt = 10
radius = 1
points = 10        # points in the circle we draw

#move_to(vehicle,0,radius,5)
for i in range(0, points):
    degrees = (i/points)*360
    radians = (math.pi/180)*degrees
    x =  radius * math.cos(radians)
    y =  radius * math.sin(radians)
    move_to(vehicle,x, y,5)

write_log_message(f" Coming Home")
vehicle.mode = VehicleMode ("RTL")
time.sleep(3) 

vehicle.close()
