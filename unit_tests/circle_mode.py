
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

#set_a(20)

create_log_file(os.path.dirname(os.path.abspath(__file__)),  os.path.splitext(os.path.basename(__file__))[0]) 

vehicle = connect(parse_connect(), wait_ready=False)
#set_home_to_zero(vehicle)
arm_and_takeoff(vehicle, 10)

vehicle.parameters['CIRCLE_OPTIONS']=1 # face the direction of the travel 
vehicle.parameters['CIRCLE_RADIUS']=2000 # set the parameter of the radius of the circle 
vehicle.mode    = VehicleMode("CIRCLE")
time.sleep(10)

write_log_message(f" Coming Home")
vehicle.mode = VehicleMode ("RTL")
time.sleep(3) 

vehicle.close()
