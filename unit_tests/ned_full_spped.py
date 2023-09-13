
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
# vehicle = connect (parse_connect(), wait_ready=False) # for simulation 
vehicle = connect("/dev/serial0", baud= 921600,  wait_ready=False,rate=10) # for raspberry p
#vehicle = connect("/dev/ttyUSB0", baud= 57600,  wait_ready=False, rate=10) #for telemetry
vehicle.wait_ready(True, raise_exception=False) # for raspberry pi & telemetry only 
signal.signal(signal.SIGINT, interrupt)

set_a(3)
#set_home_to_zero(vehicle)
drone= Drone(0.0,0.0,3) # drone at the sink 
arm_and_takeoff(vehicle, drone.hight)
time.sleep(5) # needed to stable and set EKF 

#vehicle.groundspeed =5

print( "come move_to_poition 1")

x=3
y=0
move_to_poition(vehicle,x,y,drone.hight) #need to set the ground velocity 
time.sleep(8)

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
move_to_stages_long(vehicle,x,y,10)
time.sleep(1)


write_log_message(f" Coming Home")
vehicle.mode = VehicleMode ("LAND")
#time.sleep(5) 

vehicle.close()
