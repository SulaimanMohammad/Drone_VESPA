
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


# write_log_message(f" current_lat = {vehicle.location.global_relative_frame.lat}") 
# write_log_message(f" current_lon= {vehicle.location.global_relative_frame.lon}")


time_to_pass=4
x=2
y=0
move_to_stages_long(vehicle,x,y,time_to_pass) # go 3m in X 
print(" move on x to" ,x , " in speed of",  x/float(time_to_pass) )
wait_and_hover(vehicle, 2)


x=-2
y=0
move_to_stages_long(vehicle,x,y,time_to_pass) # go 3m in X 
print(" move on x to" ,x , " in speed of",  x/float(time_to_pass) )
wait_and_hover(vehicle, 2)

write_log_message(f" Coming Home")
vehicle.mode = VehicleMode ("LAND")
time.sleep(2) 

vehicle.close()
