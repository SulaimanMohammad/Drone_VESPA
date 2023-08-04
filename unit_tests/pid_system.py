from dronekit import connect, VehicleMode, mavutil
from simple_pid import PID
import math
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

        
def normalize_angle(angle):
    while angle < 0:
        angle += 360
    while angle >= 360:
        angle -= 360
    return angle


create_log_file(os.path.dirname(os.path.abspath(__file__)),  os.path.splitext(os.path.basename(__file__))[0]) 
global vehicle
# Connect to the drone
vehicle = connect(parse_connect(), wait_ready=False)
drone= Drone(0.0,0.0,2) # drone at the sink

arm_and_takeoff(vehicle,drone.hight)


des_heading= 0
current_yaw = normalize_angle(vehicle.heading)
while current_yaw >5 :

    current_yaw = normalize_angle(vehicle.heading)
    print("Current Heading: %s" % current_yaw)

    # Check if the current yaw is close to 0 degrees (within a tolerance)
    tolerance = 5  # degrees
    if abs(current_yaw) > tolerance:
        set_yaw(vehicle, 0)
    time.sleep(0.1)
    
time.sleep(5)
print( "yaw" ,math.degrees(vehicle.attitude.yaw))




# Define PID controllers
pid_yaw = PID(1, 0, 0) # Tune these values
pid_speed = PID(1, 0,0) # Tune these values

# Desired direction and speed
desired_direction = math.radians(0) # Example: 45 degrees
desired_speed = 2 # Example: 1 m/s

# Function to send velocity
def send_velocity(vx, vy, vz, altitude):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111, 0, 0, altitude,
        vx, vy, vz, 0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg)

# Function to get the current yaw direction
def get_current_yaw():
    # Get the current yaw angle from the drone's attitude
    return vehicle.attitude.yaw

# Main control loop
vehicle.mode = VehicleMode('GUIDED')

start_time = time.time()
duration = 10 # seconds
while time.time() - start_time < duration: #not_condition_to_stop

    # Get the current yaw direction
    current_yaw = get_current_yaw()

    # Calculate the yaw error
    yaw_error = desired_direction - current_yaw

    # Get the current speed (this will need to be implemented based on your specific sensors)
    current_speed = vehicle.velocity[0] #get_current_speed()
    print( "yaw" ,math.degrees(vehicle.attitude.yaw), " current_speed", current_speed)

    # Calculate the speed error
    speed_error = desired_speed - current_speed

    # Compute control inputs using PID controllers
    #yaw_control = pid_yaw(yaw_error)
    speed_control = pid_speed(speed_error)

    # Compute the velocity components based on the yaw control input
    velocity_x = speed_control * math.cos(vehicle.attitude.yaw)
    velocity_y = speed_control * math.sin(vehicle.attitude.yaw)
    velocity_z = 0 # Assuming no vertical movement

    # Send the velocity command
    send_velocity(velocity_x, velocity_y, velocity_z, drone.hight)
    # Sleep before the next iteration
    time.sleep(0.1)

# Land or other finishing actions
vehicle.mode = VehicleMode('LAND')
time.sleep(2) 

vehicle.close()