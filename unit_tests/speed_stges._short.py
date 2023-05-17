
import os
import datetime
import sys
# Get the parent directory path
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
from expan import *
from done_ardupilot import *


create_log_file(os.path.dirname(os.path.abspath(__file__)),  os.path.splitext(os.path.basename(__file__))[0]) 

vehicle = connect(parse_connect(), wait_ready=False)
set_home_to_zero(vehicle)
arm_and_takeoff(vehicle, 10)

set_a(10)
drone= Drone(0.0,0.0) # drone at the sink 

write_log_message(f" current_lat = {vehicle.location.global_relative_frame.lat}") 
write_log_message(f" current_lon= {vehicle.location.global_relative_frame.lon}")

# move randomaly far from the sink
dir = 1 # go in x or east 
write_log_message(f"Go to S{dir}")
x,y= drone.direction(dir)
move_to_stages_short(vehicle,x,y)

write_log_message(f" current_lat = {vehicle.location.global_relative_frame.lat}") 
write_log_message(f" current_lon= {vehicle.location.global_relative_frame.lon}")
write_log_message(f"calculated distance y,x = {calculate_relative_pos(vehicle)}")
write_log_message(f"supposed distanc x,y = {drone.update_location(x,y)}")
time.sleep(60)


write_log_message(f" Coming Home")
vehicle.mode = VehicleMode ("RTL")
time.sleep(10) 

vehicle.close()
