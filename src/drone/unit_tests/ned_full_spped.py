
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

wait_and_hover(vehicle, 2)

vehicle.groundspeed =5

print( "come move_to_poition 1")

x=3
y=0
move_to_poition(vehicle,x,y,drone_hight) #need to set the ground velocity 
wait_and_hover(vehicle, 2)

print( "come move_to_poition 2")
x=0
y=0
move_to_poition(vehicle,x,y, drone_hight)
wait_and_hover(vehicle, 2)

x=3
y=0
move_to_stages_long(vehicle,x,y, 10)

print( "come back")
x=-3
y=0
move_to_stages_long(vehicle,x,y,10)
wait_and_hover(vehicle, 2)


write_log_message(f" Coming Home")
vehicle.mode = VehicleMode ("LAND")
#time.sleep(5) 

vehicle.close()
