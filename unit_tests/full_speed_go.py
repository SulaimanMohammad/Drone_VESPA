
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
        
def normalize_angle(angle):
    while angle < 0:
        angle += 360
    while angle >= 360:
        angle -= 360
    return angle

create_log_file(os.path.dirname(os.path.abspath(__file__)),  os.path.splitext(os.path.basename(__file__))[0]) 
global vehicle
# vehicle = connect(parse_connect(), wait_ready=False)
vehicle = connect("/dev/serial0", baud= 921600,  wait_ready=False)
vehicle.wait_ready(True, raise_exception=False)
signal.signal(signal.SIGINT, interrupt)

set_a(3)

drone= Drone(0.0,0.0,2) # drone at the sink
#set_home_to_zero(vehicle)
arm_and_takeoff(vehicle,drone.hight)
print( "Takeoff and wait 2 sec ")
des_heading= 0
print( "face_north ")

#     # Check the current altitude
#     current_altitude = vehicle.location.global_relative_frame.alt

#     # If the altitude is not at the desired level, adjust it
#     if current_altitude != drone.hight:
#         set_altitude(vehicle,drone.hight)
#     print( "yaw" ,math.degrees(vehicle.attitude.yaw))
#     # Add a delay to prevent excessive updates
#     time.sleep(1)
# Main loop to monitor yaw and keep it facing north


face_north(vehicle)
print( "yaw" ,math.degrees(vehicle.attitude.yaw))

# # Check the current altitude
# current_altitude = vehicle.location.global_relative_frame.alt

# # If the altitude is not at the desired level, adjust it
# if current_altitude != drone.hight:
#     set_altitude(vehicle,drone.hight)

print( "loiter is on")
vehicle.mode    = VehicleMode("LOITER") #loiter mode and hover in your place 
time.sleep(5)
vehicle.mode     = VehicleMode("GUIDED")

# write_log_message(f" current_lat = {vehicle.location.global_relative_frame.lat}") 
# write_log_message(f" current_lon= {vehicle.location.global_relative_frame.lon}")

# # move randomaly far from the sink
# dir = 1 # go in x or east 
# write_log_message(f"Go to S{dir}")
# #x,y= drone.direction(dir)
time_to_pass=4
x=2
y=0
move_to_flush(vehicle,x,y,drone.hight,time_to_pass) # go 3m in X 
print(" move on x to" ,x , " in speed of",  x/float(time_to_pass) )

face_north(vehicle)
# vehicle.mode    = VehicleMode("LOITER") #loiter mode and hover in your place 
time.sleep(5)
vehicle.mode     = VehicleMode("GUIDED")


x=-2
y=0
move_to_flush(vehicle,x,y,drone.hight,time_to_pass) # go 3m in X 
print(" move on x to" ,x , " in speed of",  x/float(time_to_pass) )

# write_log_message(f" current_lat = {vehicle.location.global_relative_frame.lat}") 
# write_log_message(f" current_lon= {vehicle.location.global_relative_frame.lon}")
# write_log_message(f"calculated distance y,x = {calculate_relative_pos(vehicle)}")
# write_log_message(f"supposed distanc x,y = {drone.update_location(x,y)}")
# time.sleep(10)


write_log_message(f" Coming Home")
vehicle.mode = VehicleMode ("LAND")
time.sleep(2) 

vehicle.close()
