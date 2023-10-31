from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import os
import datetime
import sys
import signal
# Get the parent directory path
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
from drone_ardupilot import *


speed= 2
distance= 6

class FileWriter:
    def __init__(self, filename):
        self.file = open(filename, 'w')
    
    def write(self, *args):
        message = ' '.join(map(str, args))
        print(message, file=self.file)
    
    def close(self):
        self.file.close()

# Usage
logger = FileWriter("output.txt")

def interrupt(signal_num, frame):
    print("Interrupted!")
    global vehicle
    if vehicle is not None:
        vehicle.remove_attribute_listener('velocity', on_velocity)
        vehicle.mode = VehicleMode ("LAND")
        time.sleep(3) 
        vehicle.close()
        sys.exit()


new_velocity_data = threading.Event()
def on_velocity(self, attribute_name, value):
    global velocity_listener
    global last_event_time
    global interval_between_events
    # Record current time
    current_time = time.time()
    # If this is not the first event, print the time difference
    if last_event_time is not None:
        interval_between_events = current_time - last_event_time
        logger.write(f"Time between two events: {interval_between_events} seconds")

    # Update the last event time
    last_event_time = current_time

    # Signal that new data is available
    velocity_components = value
    # Convert string values to float
    velocity_listener = [float(v) for v in velocity_components]
    logger.write(velocity_listener)
    new_velocity_data.set()

create_log_file()
global vehicle

#vehicle = connect (parse_connect(), wait_ready=False, rate=15) # for simulation 
vehicle = connect("/dev/serial0", baud= 921600,  wait_ready=False) # for raspberry pi
#vehicle = connect("/dev/ttyUSB0", baud= 57600,  wait_ready=False, rate=10) #for telemetry 
vehicle.wait_ready(True, raise_exception=False) # for raspberry pi & telemetry only 
signal.signal(signal.SIGINT, interrupt)

drone_hight=2 
arm_and_takeoff(vehicle,drone_hight)
logger.write( "Takeoff and wait 2 sec")

#loiter mode and hover in your place 
vehicle.mode    = VehicleMode("LOITER") #loiter mode and hover in your place 
time.sleep(2)
vehicle.mode     = VehicleMode("GUIDED")

set_data_rate(vehicle, 20)


angl_dir= 90#-45

desired_vel_x = speed
desired_vel_y=0.0
desired_vel_z=0.0
remaining_distance= distance 
# Set mode to GUIDED
if vehicle.mode != "GUIDED":
    vehicle.mode = VehicleMode("GUIDED")

roll = vehicle.attitude.roll
pitch = vehicle.attitude.pitch

velocity_listener=0
interval_between_events=0
last_event_time=None

[ angl_dir, velocity_direction ]= convert_angle_to_set_dir(vehicle, angl_dir)
set_yaw_to_dir_PID( vehicle, angl_dir)

vehicle.add_attribute_listener('velocity', on_velocity)
start_time = time.time()
start_control_timer=0
previous_velocity_x=0 
while remaining_distance >= 0.1:
    new_velocity_data.wait()
    
    logger.write( "---------------------------------------------------------------------")
    
    # Get current velocities from NED frame to body 
    velocity_body   =ned_to_body(vehicle,velocity_listener )
    velocity_current_x=(velocity_body[0])
    velocity_current_y=(velocity_body[1])
    velocity_current_z=(velocity_body[2])

    #if (time.time() - start_control_timer > 0.5) or (abs(error_vel_x) > abs(error_vel_x_prev) and acc==True ) or (abs(error_vel_x) < abs(error_vel_x_prev) and acc==False ) or abs(velocity_y -desired_vel_y) > 0.1 or abs(altitude_rate- desired_vel_z)> 0.5:
    if (time.time() - start_control_timer > 0.1):
        
        logger.write( "                              controle                               ")
        send_control_body(vehicle, desired_vel_x, desired_vel_y, desired_vel_z)
        start_control_timer= time.time()
    
    remaining_distance= remaining_distance - (float(interval_between_events* abs( (velocity_current_x + previous_velocity_x)/2.0 )))
    
    logger.write( "\nvx ",velocity_current_x , "vy",velocity_current_y, "vz",velocity_current_z )
    logger.write( "time", time.time() - start_time , "distance left : ",remaining_distance, "dis speed", desired_vel_x,"\n" )
    
    previous_velocity_x= velocity_current_x # save the current for next iteration 
    
    # Clear the event so we can wait for the next update
    new_velocity_data.clear()

logger.write("STOP")
send_control_body(vehicle, 0, 0, 0) # stop 
vehicle.mode    = VehicleMode("LOITER") #loiter mode and hover in your place 
time.sleep(2)
vehicle.remove_attribute_listener('velocity', on_velocity)
vehicle.mode     = VehicleMode("GUIDED")

# write_log_message(f" Coming Home")
vehicle.mode = VehicleMode ("LAND")
time.sleep(2) 

# Close connection
vehicle.close()
