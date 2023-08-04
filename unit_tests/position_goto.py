
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

def goto_position_target_local_ned(north, east, down):
    """	
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
    location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative 
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
    ignored. For more information see: 
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.

    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # send command to vehicle
    vehicle.send_mavlink(msg)



global vehicle
#vehicle = connect("/dev/ttyUSB0", wait_ready=False,baud=57600)
vehicle = connect(parse_connect(), wait_ready=False)
#vehicle.wait_ready(True, raise_exception=False)
signal.signal(signal.SIGINT, interrupt)

set_a(3)
drone= Drone(0.0,0.0, 2) # drone at the sink 
#set_home_to_zero(vehicle)
arm_and_takeoff(vehicle, drone.hight)
time.sleep(2) # needed to stable and set EKF 

# print("Set groundspeed to 15m/s (max).")
# vehicle.groundspeed = 15
# print("Set airspeed to 10m/s (max).")
# vehicle.airspeed = 10
x=3
y=0
move_to_poition(vehicle,x,y ,-drone.hight)
time.sleep(20)
goto_position_target_local_ned(0,0,-drone.hight)
time.sleep(20)

vehicle.mode = VehicleMode ("LAND")
time.sleep(20) 

vehicle.close()

# from __future__ import print_function

# from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
# from pymavlink import mavutil # Needed for command message definitions
# import time
# import math


# #Set up option parsing to get connection string
# import argparse  
# parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
# parser.add_argument('--connect', 
#                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
# args = parser.parse_args()

# connection_string = args.connect
# sitl = None


# #Start SITL if no connection string specified
# if not connection_string:
#     import dronekit_sitl
#     sitl = dronekit_sitl.start_default()
#     connection_string = sitl.connection_string()


# # Connect to the Vehicle
# print('Connecting to vehicle on: %s' % connection_string)
# vehicle = connect(connection_string, wait_ready=False )


# def arm_and_takeoff(aTargetAltitude):
#     """
#     Arms vehicle and fly to aTargetAltitude.
#     """

#     print("Basic pre-arm checks")
#     # Don't let the user try to arm until autopilot is ready
#     while not vehicle.is_armable:
#         print(" Waiting for vehicle to initialise...")
#         time.sleep(1)

        
#     print("Arming motors")
#     # Copter should arm in GUIDED mode
#     vehicle.mode = VehicleMode("GUIDED")
#     vehicle.armed = True

#     while not vehicle.armed:      
#         print(" Waiting for arming...")
#         time.sleep(1)

#     print("Taking off!")
#     vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

#     # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
#     #  after Vehicle.simple_takeoff will execute immediately).
#     while True:
#         print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
#         if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
#             print("Reached target altitude")
#             break
#         time.sleep(1)


# #Arm and take of to altitude of 5 meters
# arm_and_takeoff(5)

# def goto_position_target_local_ned(north, east, down):
#     """	
#     Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified 
#     location in the North, East, Down frame.

#     It is important to remember that in this frame, positive altitudes are entered as negative 
#     "Down" values. So if down is "10", this will be 10 metres below the home altitude.

#     Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
#     ignored. For more information see: 
#     http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned

#     See the above link for information on the type_mask (0=enable, 1=ignore). 
#     At time of writing, acceleration and yaw bits are ignored.

#     """
#     msg = vehicle.message_factory.set_position_target_local_ned_encode(
#         0,       # time_boot_ms (not used)
#         0, 0,    # target system, target component
#         mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
#         0b0000111111111000, # type_mask (only positions enabled)
#         north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
#         0, 0, 0, # x, y, z velocity in m/s  (not used)
#         0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
#         0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
#     # send command to vehicle
#     vehicle.send_mavlink(msg)

# print("SQUARE path using SET_POSITION_TARGET_LOCAL_NED and position parameters")
# DURATION = 20 #Set duration for each segment.

# print("North 50m, East 0m, 10m altitude for %s seconds" % DURATION)
# goto_position_target_local_ned(5,0,-10)
# # NOTE that this has to be called after the goto command as first "move" command of a particular type
# # "resets" ROI/YAW commands
# time.sleep(DURATION)

# # print("North 50m, East 50m, 10m altitude")
# # goto_position_target_local_ned(50,50,-10)
# # time.sleep(DURATION)

# print("North 0m, East 50m, 10m altitude")
# goto_position_target_local_ned(0,0,-10)
# time.sleep(DURATION)

# # print("North 0m, East 0m, 10m altitude")
# # goto_position_target_local_ned(0,0,-10)
# # time.sleep(DURATION)

# vehicle.mode = VehicleMode ("LAND")
# time.sleep(5) 

# vehicle.close()