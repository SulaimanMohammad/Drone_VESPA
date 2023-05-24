
import os
import datetime
import sys
# Get the parent directory path
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
from expan import *
from done_ardupilot import *
import math 

camera_image_width=15
distance=math.sin(math.radians(60))* camera_image_width


create_log_file(os.path.dirname(os.path.abspath(__file__)),  os.path.splitext(os.path.basename(__file__))[0]) 

vehicle = connect(parse_connect(), wait_ready=False)
set_home_to_zero(vehicle)
arm_and_takeoff(vehicle, 10)

set_a(10)
drone= Drone(0.0,0.0) # drone at the sink 

write_log_message(f" current_lat = {vehicle.location.global_relative_frame.lat}") 
write_log_message(f" current_lon= {vehicle.location.global_relative_frame.lon}")

new_radious= a-distance/2
x=0
y=new_radious
move_to(vehicle,x,y)
while new_radious >0: 
    # if x >0 north , y> east 
    # go to the vertex1 

    x= math.sin(math.radians(60))* (new_radious)
    y=- math.cos(math.radians(60))* (new_radious)
    move_to(vehicle,x,y)

    x=0
    y=-new_radious
    move_to(vehicle,x,y)

    x=-math.sin(math.radians(60))* (new_radious)
    y=-math.cos(math.radians(60))* (new_radious)
    move_to(vehicle,x,y)

    x=-math.sin(math.radians(60))* (new_radious)
    y= math.cos(math.radians(60))* (new_radious)
    move_to(vehicle,x,y)

    x=0
    y=+new_radious
    move_to(vehicle,x,y)

    x=math.sin(math.radians(60))* (new_radious)
    y=math.cos(math.radians(60))* (new_radious)
    move_to(vehicle,x,y)
    time.sleep(1)
    new_radious=new_radious-(distance/2)
    x=0
    y=-distance/2
    move_to(vehicle,x,y)



write_log_message(f" current_lat = {vehicle.location.global_relative_frame.lat}") 
write_log_message(f" current_lon= {vehicle.location.global_relative_frame.lon}")
write_log_message(f"calculated distance y,x = {calculate_relative_pos(vehicle)}")
write_log_message(f"supposed distanc x,y = {drone.update_location(x,y)}")
time.sleep(2)

write_log_message(f" Coming Home")
vehicle.mode = VehicleMode ("RTL")
time.sleep(10) 

vehicle.close()
