
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
        vehicle.mode = VehicleMode ("LAND")
        time.sleep(3) 
        vehicle.close()
        sys.exit()
        
create_log_file(os.path.dirname(os.path.abspath(__file__)),  os.path.splitext(os.path.basename(__file__))[0]) 

global vehicle
vehicle = connect(parse_connect(), wait_ready=False)
# vehicle = connect("/dev/ttyUSB0", wait_ready=False,baud=57600)
# vehicle.wait_ready(True, raise_exception=False)
signal.signal(signal.SIGINT, interrupt)

set_a(3)
#set_home_to_zero(vehicle)
drone= Drone(0.0,0.0,3) # drone at the sink 
arm_and_takeoff(vehicle, drone.hight)
time.sleep(5) # needed to stable and set EKF 


# write_log_message(f" current_lat = {vehicle.location.global_relative_frame.lat}") 
# write_log_message(f" current_lon= {vehicle.location.global_relative_frame.lon}")
# print("Set groundspeed to 15m/s (max).")
# vehicle.groundspeed =5
# print("Set airspeed to 10m/s (max).")
#vehicle.airspeed = 2
# move randomaly far from the sink
dir = 1 # go in x or east 
write_log_message(f"Go to S{dir}")
#x,y= drone.direction(dir)

print( "come move_to_poition 1")

x=3
y=0
move_to_poition(vehicle,x,y,drone.hight) #need to set the ground velocity 
time.sleep(8)
#move_to_position_speed_flush(vehicle,x,y,drone.hight,20)
print( "come move_to_poition 2")
x=0
y=0
move_to_poition(vehicle,x,y, drone.hight)
time.sleep(8)

x=3
y=0
move_to_stages_long(vehicle,x,y, 10)

print( "come back")
x=-3
y=0
#move_to_flush(vehicle,x,y,drone.hight,2)
#move_to_stages_short(vehicle,x,y)
move_to_stages_long(vehicle,x,y,10)


time.sleep(1)


write_log_message(f" Coming Home")
vehicle.mode = VehicleMode ("LAND")
#time.sleep(5) 

vehicle.close()
