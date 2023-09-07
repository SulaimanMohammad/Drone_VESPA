from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
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
        vehicle.remove_attribute_listener('velocity', on_velocity)
        vehicle.mode = VehicleMode ("LAND")
        time.sleep(3) 
        vehicle.close()
        sys.exit()

# # Define a listener function to receive updates
# def attribute_listener(self, attr_name, value):
#     if attr_name == 'location.global_frame':
#         latitude = value.lat
#         longitude = value.lon
#         altitude = value.alt
#         print(f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}")

# def on_velocity(self, attribute_name, value):
#     print("Velocity: %s" % str(value))

create_log_file(os.path.dirname(os.path.abspath(__file__)),  os.path.splitext(os.path.basename(__file__))[0]) 
global vehicle

vehicle = connect (parse_connect(), wait_ready=False) # for simulation 
#vehicle = connect("/dev/serial0", baud= 921600,  wait_ready=False,rate=10) # for raspberry pi
#vehicle = connect("/dev/ttyUSB0", baud= 57600,  wait_ready=False, rate=10) #for telemetry 
#vehicle.wait_ready(True, raise_exception=False) # for raspberry pi & telemetry only 
signal.signal(signal.SIGINT, interrupt)

set_a(3)
# vehicle.add_attribute_listener('velocity', on_velocity)
# Add the listener to the vehicle's attribute
# vehicle.add_attribute_listener('location.global_frame', attribute_listener)

drone= Drone(0.0,0.0,2) # drone at the sink 
#set_home_to_zero(vehicle)
arm_and_takeoff(vehicle,drone.hight)
print( "Takeoff and wait 2 sec")

vehicle.mode    = VehicleMode("LOITER") #loiter mode and hover in your place 
time.sleep(2)
vehicle.mode     = VehicleMode("GUIDED")
# face_north(vehicle)

set_data_rate(vehicle, 20)

distance=2
angl_dir= 90#-45
# angle dir in degree 
move_body_PID(vehicle, drone.hight, angl_dir, distance)

vehicle.mode    = VehicleMode("LOITER") #loiter mode and hover in your place 
time.sleep(2)
vehicle.mode     = VehicleMode("GUIDED")

print( "move to the other direction")

angl_dir= -90#135
move_body_PID(vehicle,drone.hight,  angl_dir, distance)


write_log_message(f" Coming Home")
vehicle.mode = VehicleMode ("LAND")
time.sleep(2) 
# vehicle.remove_attribute_listener('location.global_frame', attribute_listener)

# Close connection
vehicle.close()
