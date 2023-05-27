
import os
import datetime
import sys
# Get the parent directory path
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
from expan import *
from drone_ardupilot import *
import math 

camera_image_width=5*sqrt(3)
#set_a(20)

create_log_file(os.path.dirname(os.path.abspath(__file__)),  os.path.splitext(os.path.basename(__file__))[0]) 

vehicle = connect(parse_connect(), wait_ready=False)
set_home_to_zero(vehicle)
arm_and_takeoff(vehicle, 10)

drone= Drone(0.0,0.0) # drone at the sink 

write_log_message(f" current_lat = {vehicle.location.global_relative_frame.lat}") 
write_log_message(f" current_lon= {vehicle.location.global_relative_frame.lon}")
#time is based on the time of image processing 
scan_hexagon(vehicle, drone, camera_image_width,30)

write_log_message(f" current_lat = {vehicle.location.global_relative_frame.lat}") 
write_log_message(f" current_lon= {vehicle.location.global_relative_frame.lon}")
write_log_message(f"calculated distance y,x = {calculate_relative_pos(vehicle)}")
write_log_message(f"supposed distanc x,y = {drone.positionX,drone.positionY }")
time.sleep(2)

write_log_message(f" Coming Home")
vehicle.mode = VehicleMode ("RTL")
time.sleep(10) 

vehicle.close()
