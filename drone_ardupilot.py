import dronekit_sitl
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import math
import time
import argparse
import os
import datetime
from expan import DIR_VECTORS
import inspect
import numpy as np
from simple_pid import PID


# Declare global variables for logs 
filename = " "
directory= " "

def create_log_file(dir,script_name ):
    # Get the name of the script using this module
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    # Create the log file name
    global filename
    filename = f"{script_name}_{timestamp}.log"
    
    # Set the global directory variable
    global directory
    directory = os.path.join(dir, 'log')
    if not os.path.exists(directory): # if log doesnt exist then create it 
        os.makedirs(directory)

def open_log_file():
    # Get the absolute path of the log file
    log_file_path = os.path.join(directory, filename)
    # Open the log file
    return open(log_file_path, 'a') # dont use "w" because it remove all what was before after each open

def write_log_message(message):
    with open_log_file() as log_file:
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        log_file.write(f"{timestamp}: {message}\n")

def get_current_function_name():
    # Get the frame of the calling function
    frame = inspect.currentframe().f_back
    # Get the name of the calling function from the frame
    function_name = frame.f_code.co_name
    return function_name

def parse_connect():
    write_log_message (f"{get_current_function_name()} called:") 
    parser=argparse.ArgumentParser (description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string = args.connect
    #print ("Connection to the vehicle on %s"%connection_string)
    # Connect to the Vehicle (in this case a simulator running the same computer)
    return connection_string

def arm_and_takeoff(self, aTargetAltitude):
    write_log_message (f"{get_current_function_name()} called:") 

    """
    Arms vehicle and fly to aTargetAltitude.
    """
    if self.mode.name == "INITIALISING":
        print ("initialise the self") 
        write_log_message ("initialise the self") 

    print("Basic pre-arm checks") 
    # TODO do not try to take off if the alituduid is not zero 
    # Don't try to arm until autopilot is ready
    #TODO check that gps lock before the fly by listening to the raw data of the channel
    #GPs usually it is done by the system  of pixhawc 
    # TODO check that the battery is enough to fly
    while not self.is_armable:
        print ("Waiting for self to initialise and Armability...")
        write_log_message ("Waiting for self to initialise...") 
        time.sleep(1)

    print ("Arming motors")
    write_log_message ("Arming motors")
    # Copter should arm in GUIDED mode
    self.mode    = VehicleMode("GUIDED")
    self.armed   = True
    print(" Battery: %s" % self.battery)

    # Confirm vehicle armed before attempting to take off
    while not self.armed:
        print (" Waiting for arming...")
        write_log_message (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off starts") 
    write_log_message ("Taking off starts")
    self.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    #takeoff is asynchronous and can be interrupted if another command arrives before it reaches the target altitude so
    # check that arrived by reading the alitude relative from the ground 
    while True:
        print (" Altitude: ", self.location.global_relative_frame.alt)
        write_log_message (f" Altitude: {self.location.global_relative_frame.alt}")
        #Break and return from function just below target altitude.
        if self.location.global_relative_frame.alt>=aTargetAltitude*0.95:  # arrived to 95% of the altitude s
            print ("Reached target altitude")
            write_log_message ("Reached target altitude")
            break
        time.sleep(1)

def calculate_relative_pos(self):
    write_log_message (f"{get_current_function_name()} called:")
    relative_x=0
    relative_y=0
    relative_z=0
    for i in range(3):
        # Get the drone's current GPS coordinates
        current_lat = self.location.global_relative_frame.lat
        current_lon = self.location.global_relative_frame.lon
        current_alt = self.location.global_relative_frame.alt

        # Calculate the drone's position relative to the home position
        home_lat = self.home_location.lat
        home_lon = self.home_location.lon
        home_alt = self.home_location.alt
        relative_x += (current_lat - home_lat) * 1.113195e5
        relative_y += (current_lon - home_lon) * 1.113195e5
        relative_z += current_alt - home_alt
        time.sleep(0.1)
        # Print the drone's position relative to the home position
        return [relative_x/3,relative_y/3, relative_z/3]


def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, altitude,  duration):
    write_log_message (f"{get_current_function_name()} called:")
    msg = self.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # MAV_FRAME_LOCAL_OFFSET_NED, 
        0b111111000111,#0b111111000111, #0b110111000111, #, # type_mask (only speeds enabled)
        0, 0, altitude, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    # for x in range(0,duration):
    #     self.send_mavlink(msg)
    #     #calculate_relative_pos()
    #     time.sleep(1)
    # more accurate than using for loop and sleep because duration could be a float number 
    start_time = time.time()
    while time.time() - start_time < duration:
        self.send_mavlink(msg)
        time.sleep(0.1)


def send_ned_position(self, north, east, down):
    write_log_message (f"{get_current_function_name()} called:")
    msg = self.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    # more accurate than using for loop and sleep because duration could be a float number 
    self.send_mavlink(msg)
    #self.flush()

def send_ned_velocity_stages_long(self, velocity_x, velocity_y, velocity_z, duration):
    write_log_message (f"{get_current_function_name()} called:")
    stages = 6  # Number of stages

    # Calculate distance and time for each stage
    stage_duration = duration / stages

    # Loop through stages
    for stage in range(stages):
        # Calculate speed for the first stage 25% of the speed 
        if stage == 0 or stage == stages - 1:  # First and last stage (25% and 25%)
            stage_speed_x = velocity_x * 0.25
            stage_speed_y = velocity_y * 0.25
            stage_speed_z = velocity_z * 0.25

        elif stage ==2 or stage ==3:  # Middle stage (100%)
            stage_speed_x = velocity_x 
            stage_speed_y = velocity_y 
            stage_speed_z = velocity_z 
        else:  # Intermediate stages (50%)
            stage_speed_x = velocity_x * 0.5
            stage_speed_y = velocity_y * 0.5
            stage_speed_z = velocity_z * 0.5

        # Calculate duration for the current stage
        if stage == 0 or stage == stages - 1:  # First and last stage (25% and 25%)
            stage_time = stage_duration * 4.0 # since it speed of 25% then you need 4 times the duration of the satge to go the distance 
        elif stage ==2 or stage ==3:  # Middle stage (100%)
            stage_time = stage_duration
        else:  # Intermediate stages (50%)
            stage_time = stage_duration *2 # same herer you need 2 times the duration of the stage since you need only 50% of speed so double the time 

        # Calculate duration for the current stage faster 
        if stage == 0 or stage == stages - 1:  # First and last stage (25% and 25%)
            stage_time = stage_duration * 2.0 # since it speed of 25% then you need 4 times the duration of the satge to go the distance 
        elif stage ==2 or stage ==3:  # Middle stage (100%)
            stage_time = stage_duration*2
        else:  # Intermediate stages (50%)
            stage_time = stage_duration *1 # same herer you need 2 times the duration of the stage since you need only 50% of speed so double the time 

        # Create and send the MAVLink message for the current stage
        msg = self.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b110111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            stage_speed_x, stage_speed_y, stage_speed_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        
        # print(f"Stage {stage+1}:, Duration={stage_duration}s"," speed_x= ", stage_speed_x, "  speed_y= ", stage_speed_y)

        # Send command to vehicle on 1 Hz cycle for the current stage's duration
        start_time = time.time()
        while time.time() - start_time < stage_time:
            self.send_mavlink(msg)
            time.sleep(0.1)

def send_ned_velocity_stages_short(self, velocity_x, velocity_y, velocity_z, duration):
    write_log_message (f"{get_current_function_name()} called:")
    stages = 6  # Number of stages

    # Calculate distance and time for each stage
    stage_duration = duration / stages

    # Loop through stages
    for stage in range(stages):
        # Calculate speed for the first stage 25% of the speed 
        if stage == 0 or stage == stages - 1:  # First and last stage (25% and 25%)
            stage_speed_x = velocity_x * 0.25
            stage_speed_y = velocity_y * 0.25
            stage_speed_z = velocity_z * 0.25

        elif stage ==2 or stage ==3:  # Middle stage (100%)
            stage_speed_x = velocity_x 
            stage_speed_y = velocity_y 
            stage_speed_z = velocity_z 
        else:  # Intermediate stages (50%)
            stage_speed_x = velocity_x * 0.5
            stage_speed_y = velocity_y * 0.5
            stage_speed_z = velocity_z * 0.5

        # Calculate duration for the current stage faster 
        if stage == 0 or stage == stages - 1:  # First and last stage (25% and 25%)
            stage_time = stage_duration * 2.0 # since it speed of 25% then you need 4 times the duration of the satge to go the distance 
        elif stage ==2 or stage ==3:  # Middle stage (100%)
            stage_time = stage_duration*2
        else:  # Intermediate stages (50%)
            stage_time = stage_duration *1 # same herer you need 2 times the duration of the stage since you need only 50% of speed so double the time 

        # Create and send the MAVLink message for the current stage
        msg = self.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
            0b110111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            stage_speed_x, stage_speed_y, stage_speed_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # Send command to vehicle on 1 Hz cycle for the current stage's duration
        start_time = time.time()
        while time.time() - start_time < stage_time:
            self.send_mavlink(msg)
            time.sleep(0.5)

def send_ned_velocity_flush(self, velocity_x, velocity_y, velocity_z, altitude, duration):
    write_log_message (f"{get_current_function_name()} called:")
    msg = self.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # MAV_FRAME_LOCAL_OFFSET_NED, 
        0b0000111111000011,#0b111111000111, #0b110111000111, #, # type_mask (only speeds enabled)
        0, 0, altitude, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    # for x in range(0,duration):
    #     self.send_mavlink(msg)
    #     #calculate_relative_pos()
    #     time.sleep(1)
    # more accurate than using for loop and sleep because duration could be a float number 
    self.send_mavlink(msg)
    #self.flush()

# this function will calculate the speed and the time and call send_ned_velocity 
def move_to_flush(self, x , y, altitude, time_passed=0): 
    write_log_message (f"{get_current_function_name()} called:")
    # need to set speed constant and we change the time 
    # the north is the y access in the calaulation of the hex and the east is the x 
    # example reived x=a, y=b 
    # then the move to the nourth by the value of b ( y in calculation access)
    north= y  # on y
    east= x #on x 
    down=0 # stay in the same hight 
    if time_passed==0:
        total_time= min_time_safe()
    else:
        total_time=time_passed
    # counter=0 
    # while counter< time_passed:
    #     send_ned_velocity_flush(self, north/float(total_time), east/float(total_time), down, total_time)
    #     time.sleep(1)
    #     counter=counter+1
    one_time=0
    start_time = time.time()
    while time.time() - start_time < time_passed:            
        send_ned_velocity_flush(self, north/float(total_time), east/float(total_time),0,-1*altitude, total_time)
        velocity_current_x = (self.velocity[0])
        velocity_current_y = (self.velocity[1])
        print( "Befor PID loop velocity[0] ",velocity_current_x , "velocity[1]",velocity_current_y, "yaw", math.degrees(self.attitude.yaw) )
        time.sleep(1)

def move_to_poition(self, x , y, z) :
    write_log_message (f"{get_current_function_name()} called:")
    north= y  # on y
    east= x #on x 
    down= -z # stay in the same hight 
    send_ned_position(self, north, east, down)

# this function will calculate the speed and the time and call send_ned_velocity 
def move_to(self, x , y, time_passed=0): 
    write_log_message (f"{get_current_function_name()} called:")
    # need to set speed constant and we change the time 
    # the north is the y access in the calaulation of the hex and the east is the x 
    # example reived x=a, y=b 
    # then the move to the nourth by the value of b ( y in calculation access)
    north= y  # on y
    east= x #on x 
    down=0 # stay in the same hight 
    if time_passed==0:
        total_time= min_time_safe()
    else:
        total_time=time_passed
    #print(" speed on x=",round(north/float(total_time),4), "on y=", round(east/float(total_time),4) )
    send_ned_velocity(self, north/float(total_time), east/float(total_time), down, total_time)
    #send_ned_velocity_stages(self,north/float(total_time) ,east/float(total_time), down, total_time)
    # you can not have one speed for both and only one time for both , so we fix time and change the speed 

def move_to_stages_long(self, x , y, time_passed=0): 
    write_log_message (f"{get_current_function_name()} called:")
    # need to set speed constant and we change the time 
    # the north is the y access in the calaulation of the hex and the east is the x 
    # example reived x=a, y=b 
    # then the move to the nourth by the value of b ( y in calculation access)
    north= y  # on y
    east= x #on x 
    down=0 # stay in the same hight 
    if time_passed==0:
        total_time= min_time_safe()
    else:
        total_time=time_passed
    send_ned_velocity_stages_long(self,north/float(total_time) ,east/float(total_time), down, total_time)

def move_to_stages_short(self, x , y,time_passed=0): 
    write_log_message (f"{get_current_function_name()} called:")
    # need to set speed constant and we change the time 
    # the north is the y access in the calaulation of the hex and the east is the x 
    # example reived x=a, y=b 
    # then the move to the nourth by the value of b ( y in calculation access)
    north= y  # on y
    east= x #on x 
    down=0 # stay in the same hight 
    if time_passed==0:
        total_time= min_time_safe()
    else:
        total_time=time_passed
    send_ned_velocity_stages_short(self,north/float(total_time) ,east/float(total_time), down, total_time)

def set_home_to_zero(self):
    write_log_message (f"{get_current_function_name()} called:")

    home_position_cmd = self.message_factory.command_long_encode(
    0, 0,                             # target system, target component
    mavutil.mavlink.MAV_CMD_DO_SET_HOME,              # command
    0,                                # confirmation
    1,                                # set current location as home
    0, 0, 0,                      # unused parameters for this command
    0, 0, 0                           # unused parameters for this command
    )
    self.send_mavlink(home_position_cmd)

def min_time_safe():
    write_log_message (f"{get_current_function_name()} called:")
    max_speed = 5  # Maximum speed in m/s
    longest_duration = 0  # Initialize the longest duration as 0

    for position in DIR_VECTORS:
        distance = math.sqrt(position[0]**2 + position[1]**2)  # Calculate the distance to the position
        duration = distance / max_speed *0.90 # Calculate the duration to reach the position # take only 0.90% for the safty 

        if duration > longest_duration:
            longest_duration = duration
    return int(longest_duration)


def set_altitude(self, target_altitude):
    # Create a location object with the same latitude and longitude as the current location, but with the desired altitude
    target_location = LocationGlobalRelative(self.location.global_relative_frame.lat,
                                             self.location.global_relative_frame.lon,
                                             target_altitude)
    # Command the vehicle to go to the target location
    self.simple_goto(target_location)


'''
yaw values are fluctuating around 180 and -180 degrees, which is very close to the north direction (0 degrees) 
but represented differently.
This is a common issue in dealing with angles, as 0 degrees and 180 degrees are the same direction
 but on opposite sides of the compass.

You can handle this by normalizing the yaw angle to a value between 0 and 360 degrees 
and then checking if it's close to 0 degrees. Here's how you can modify the monitoring loop:

'''

def normalize_angle(angle):
    while angle < 0:
        angle += 360
    while angle >= 360:
        angle -= 360
    return angle

def set_yaw (self,yaw_angle, relative=False):
    if relative:
        is_relative = 1
    else:
        is_relative = 0

    # Create a MAVLink command for setting the yaw
    msg = self.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
        0, # confirmation
        yaw_angle, # param 1 - yaw angle ( in degree)
        0, # param 2 - yaw speed
        1, # param 3 - direction: 1 = clockwise, -1 = counterclockwise
        is_relative, # param 4 - relative offset: 1 = relative, 0 = absolute
        0, 0, 0) # param 5 ~ 7 (not used)
    # Send the command to the vehicle
    self.send_mavlink(msg) 

def face_north(self):
    #current_yaw = normalize_angle(self.heading)
    current_yaw = normalize_angle(math.degrees(self.attitude.yaw))
    tolerance =1 # degrees
    # Check if the current yaw is close to 0 degrees (within a tolerance)
    # while abs(current_yaw) > tolerance :
    while not (current_yaw <= 5 or current_yaw >= 355) :
        #current_yaw = normalize_angle(self.heading)
        set_yaw(self, 0)
        time.sleep(0.01)
        current_yaw = normalize_angle(math.degrees(self.attitude.yaw))

def face_north_hold_gps(self):
    self.mode     = VehicleMode("GUIDED")
    #current_yaw = normalize_angle(self.heading)
    current_yaw = normalize_angle(math.degrees(self.attitude.yaw))
    tolerance = 5 # degrees
    # Get the current position to hold
    hold_position = self.location.global_relative_frame
    # Check if the current yaw is close to 0 degrees (within a tolerance)
    while not (current_yaw <= 5 or current_yaw >= 355) :
        #current_yaw = normalize_angle(self.heading)
        set_yaw(self, 0)
        # Hold position
        time.sleep(0.01)
        current_yaw = (math.degrees(self.attitude.yaw))
        print( "current",current_yaw )
        self.simple_goto(hold_position)



def face_north_loiter(self):
    #current_yaw = normalize_angle(self.heading)
    self.mode = VehicleMode("LOITER")
    current_yaw = normalize_angle(math.degrees(self.attitude.yaw))
    tolerance = 5 # degrees
    # Check if the current yaw is close to 0 degrees (within a tolerance)
    while not (current_yaw <= 5 or current_yaw >= 355):
        #current_yaw = normalize_angle(self.heading)
        set_yaw(self, 0)
        # Hold position
        time.sleep(0.01)
        current_yaw = normalize_angle(math.degrees(self.attitude.yaw))
    self.mode     = VehicleMode("GUIDED")



def position_control(self,target_altitude, target_latitude, target_longitude):
    # Initialize PID controllers
    altitude_pid = PID(0.1, 0.01, 0.008, setpoint=0)
    latitude_pid = PID(1, 0.1, 0.01, setpoint=0)
    longitude_pid = PID(1, 0.1, 0.01, setpoint=0)

    # Current values
    current_altitude = self.location.global_relative_frame.alt
    current_latitude = self.location.global_relative_frame.lat
    current_longitude = self.location.global_relative_frame.lon

    # Calculate errors
    altitude_error = target_altitude - current_altitude
    latitude_error = target_latitude - current_latitude
    longitude_error = target_longitude - current_longitude

    # Calculate control inputs using PID controllers
    throttle_control = altitude_pid(altitude_error)
    pitch_control = latitude_pid(latitude_error)
    roll_control = longitude_pid(longitude_error)

    # Apply control inputs
    #self.channels.overrides['3'] = throttle_control
    self.channels.overrides['1'] = pitch_control
    self.channels.overrides['2'] = roll_control


def face_north_hold_PID(self):
    # Target values
    target_altitude = self.location.global_relative_frame.alt
    target_latitude = self.location.global_relative_frame.lat
    target_longitude = self.location.global_relative_frame.lon

    self.mode     = VehicleMode("GUIDED")
    #current_yaw = normalize_angle(self.heading)
    current_yaw = normalize_angle(math.degrees(self.attitude.yaw))
    tolerance = 5 # degrees
    # Get the current position to hold
    # Check if the current yaw is close to 0 degrees (within a tolerance)
    while not (current_yaw <= 5 or current_yaw >= 355) :
        #current_yaw = normalize_angle(self.heading)
        set_yaw(self, 0)
        # Hold position
        time.sleep(0.1)
        current_yaw = (math.degrees(self.attitude.yaw))
        print( "current",current_yaw )
        position_control(self,target_altitude, target_latitude, target_longitude)


def set_yaw_to_dir(self,yaw_angle):
     #current_yaw = normalize_angle(self.heading)
    current_yaw = normalize_angle(math.degrees(self.attitude.yaw))
    yaw_angle= normalize_angle(yaw_angle) 
    tolerance = 1 # degrees
    print( "yaw frst",normalize_angle(math.degrees(self.attitude.yaw)) )
    # Check if the current yaw is close to 0 degrees (within a tolerance)
    while abs(current_yaw - yaw_angle) > tolerance:
        #current_yaw = normalize_angle(self.heading)
        set_yaw(self, yaw_angle)
        time.sleep(0.01)
        current_yaw = normalize_angle(math.degrees(self.attitude.yaw))
    
    print( "yaw frst",normalize_angle(math.degrees(self.attitude.yaw)) )


# def set_yaw_to_dir_PID(self,yaw_angle):
#      #current_yaw = normalize_angle(self.heading)
#     current_yaw = normalize_angle(math.degrees(self.attitude.yaw))
#     yaw_angle= normalize_angle(yaw_angle) 
#     tolerance = 1 # degrees
#     print( "yaw frst",normalize_angle(math.degrees(self.attitude.yaw)) )
#     # Check if the current yaw is close to 0 degrees (within a tolerance)
#     while not ( abs(current_yaw- yaw_angle) < +5 ) :
#         #current_yaw = normalize_angle(self.heading)
#         set_yaw(self, yaw_angle)
#         time.sleep(0.01)
#         current_yaw = normalize_angle(math.degrees(self.attitude.yaw))
    
#     print( "yaw frst",normalize_angle(math.degrees(self.attitude.yaw)) )


def set_yaw_PID(self, yaw_angle, yaw_speed, direction, relative=False):
    if relative:
        is_relative = 1
    else:
        is_relative = 0

    # Create a MAVLink command for setting the yaw
    msg = self.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, # command
        0, # confirmation
        yaw_angle, # param 1 - yaw angle (in degrees)
        yaw_speed, # param 2 - yaw speed (in degrees/second)
        direction, # param 3 - direction: 1 = clockwise, -1 = counterclockwise
        is_relative, # param 4 - relative offset: 1 = relative, 0 = absolute
        0, 0, 0) # param 5 ~ 7 (not used)
    # Send the command to the vehicle
    self.send_mavlink(msg)

def set_yaw_to_dir_PID(self, target_yaw, max_yaw_speed=10):
    
    kp=0.8
    ki=0.01
    kd=0.01

    # Target values
    target_altitude = self.location.global_relative_frame.alt
    target_latitude = self.location.global_relative_frame.lat
    target_longitude = self.location.global_relative_frame.lon

    pid = PID(kp, ki, kd, setpoint=target_yaw)
    pid.output_limits = (-max_yaw_speed, max_yaw_speed)  # Limits to ensure the output is within valid bounds
    pid.sample_time = 0.1  # Update interval

    while True:
        current_yaw = normalize_angle(math.degrees(self.attitude.yaw))
        error = normalize_angle(target_yaw - current_yaw)

        # Correcting error if it's shorter to turn the other way
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # Check if error is within tolerance
        if abs(error) < 5:
            break
        # Get the PID output
        yaw_speed = pid(error)
        
        # Determine the direction (clockwise or counterclockwise)
        direction = 1 if error >= 0 else -1

        # Send the yaw command
        set_yaw_PID(self, abs(error), abs(yaw_speed), direction, relative=True)
        #position_control(self,target_altitude, target_latitude, target_longitude)
        time.sleep(pid.sample_time)
    point1 = LocationGlobalRelative(target_latitude, target_longitude, target_altitude)

    self.simple_goto(point1, groundspeed=1)
    print("Yaw set to:", target_yaw)




# Send a command to control velocity and yaw
def send_control(vehicle, velocity_x, velocity_y, yaw_rate):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
        0b0000010111000111,
        0, 0, 0,
        velocity_x, velocity_y, 0,
        0, 0, 0,
        yaw_rate, 0)
    vehicle.send_mavlink(msg)

def move_PID(self, angl_dir, distance, time_needed):
    # Set mode to GUIDED
    self.mode = VehicleMode("GUIDED")
    set_yaw_to_dir_PID( self, angl_dir)
    #set_yaw_to_dir( self, angl_dir)
    print ( "current yaw in Deg ", math.degrees(self.attitude.yaw))
    # PID gains for yaw, X, and Y control
    Kp_yaw = 0.1
    Ki_yaw = 0.01
    Kd_yaw = 0.01

    Kp_vel_x = 0.1
    Ki_vel_x = 0.01
    Kd_vel_x = 0.01

    Kp_vel_y = 0.1
    Ki_vel_y = 0.01
    Kd_vel_y = 0.01

    # Errors and previous errors for PID control
    error_yaw_prev = 0
    error_vel_x_prev = 0
    error_vel_y_prev = 0

    integral_yaw = 0
    integral_vel_x = 0
    integral_vel_y = 0

    # Desired yaw and velocities
    desired_yaw = angl_dir # baed on the direction  needed 
    print ( "DEs yaw in Deg ", desired_yaw , " in rad",math.radians(desired_yaw) )
    desired_yaw = math.radians(desired_yaw)
    
    
    desired_vel_x = distance/float(time_needed-1)
    desired_vel_y = distance/float(time_needed-1)
    
    desired_vel_x = desired_vel_x * (math.cos(desired_yaw) )# x is on east for that it is cos
    desired_vel_y = desired_vel_y * ( math.sin(desired_yaw) )
    #desired_vel_y=0
    print (" Des vx", desired_vel_x ," Des vy", desired_vel_y )
    #distance=10 # 2 meter 
    #duration= 50 #
    #duration= distance/desired_vel_x
    duration= time_needed
    send_control(self, desired_vel_x, desired_vel_y, 0)
    time.sleep(1) 
    velocity_current_x = abs(self.velocity[0])
    velocity_current_y = abs(self.velocity[1])
    print( "Befor PID loop vx ",velocity_current_x , "vy",velocity_current_y, "yaw", math.degrees(self.attitude.yaw) )
    start_time = time.time()
    #duration = 10 # seconds    
    while time.time() - start_time < duration-1: #not_condition_to_stop
        # Get current yaw
        yaw_current = self.attitude.yaw

        # Yaw error and PID control
        error_yaw = desired_yaw - yaw_current
        integral_yaw += error_yaw
        derivative_yaw = error_yaw - error_yaw_prev

        yaw_rate = yaw_current + Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw
        error_yaw_prev = error_yaw

        # Get current velocities
        velocity_current_x = abs(self.velocity[0])
        velocity_current_y =abs(self.velocity[1])
        print( "vx ",velocity_current_x , "vy",velocity_current_y, "yaw", math.degrees(self.attitude.yaw) )
        # X velocity error and PID control
        error_vel_x = desired_vel_x - velocity_current_x
        integral_vel_x += error_vel_x
        derivative_vel_x = error_vel_x - error_vel_x_prev

        velocity_x = velocity_current_x + Kp_vel_x * error_vel_x + Ki_vel_x * integral_vel_x + Kd_vel_x * derivative_vel_x
        error_vel_x_prev = error_vel_x

        # Y velocity error and PID control
        error_vel_y = desired_vel_y - velocity_current_y
        integral_vel_y += error_vel_y
        derivative_vel_y = error_vel_y - error_vel_y_prev

        velocity_y = velocity_current_y + Kp_vel_y * error_vel_y + Ki_vel_y * integral_vel_y + Kd_vel_y * derivative_vel_y
        error_vel_y_prev = error_vel_y

        # Send control to the drone
        send_control(self, velocity_x, velocity_y, yaw_rate)

        # Pause before next iteration
        time.sleep(0.1)


def velocity_in_body_frame(self):
    # Retrieve the vehicle's velocity in the NED frame
    velocity_ned = np.array([self.velocity[0], self.velocity[1], self.velocity[2]])
    
    # Retrieve the vehicle's roll, pitch, and yaw angles
    roll = self.attitude.roll
    pitch = self.attitude.pitch
    yaw = self.attitude.yaw
    
    # Create the rotation matrix using the roll, pitch, and yaw angles
    R_roll = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    R_pitch = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    R_yaw = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Combine the rotations to transform from the NED frame to the body frame
    R_ned_to_body = R_roll.dot(R_pitch).dot(R_yaw)

    # Transform the velocity vector
    velocity_body = R_ned_to_body.dot(velocity_ned)

    return velocity_body


# def ned_to_body_frame(velocity_ned_x, velocity_ned_y, yaw_angle):
#     # Create a rotation matrix using the yaw angle
#     R_yaw = [
#         [math.cos(yaw_angle), -math.sin(yaw_angle)],
#         [math.sin(yaw_angle), math.cos(yaw_angle)]
#     ]

#     # Multiply the velocity in the NED frame by the rotation matrix
#     velocity_body_x = R_yaw[0][0] * velocity_ned_x + R_yaw[0][1] * velocity_ned_y
#     velocity_body_y = R_yaw[1][0] * velocity_ned_x + R_yaw[1][1] * velocity_ned_y

#     return velocity_body_x, velocity_body_y


def ned_to_body_frame(velocity_ned_x, velocity_ned_y, roll, pitch, yaw):
    # Create the rotation matrices using the roll, pitch, and yaw angles
    R_roll = np.array([
        [1, 0, 0],
        [0, math.cos(roll), -math.sin(roll)],
        [0, math.sin(roll), math.cos(roll)]
    ])

    R_pitch = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)]
    ])

    R_yaw = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw), math.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Combine the rotations to transform from the NED frame to the body frame
    R_ned_to_body = R_roll.dot(R_pitch).dot(R_yaw)

    # Transform the velocity vector
    velocity_body = R_ned_to_body.dot([velocity_ned_x, velocity_ned_y, 0])

    return velocity_body[0], velocity_body[1]

# Send a command to control velocity and yaw
def send_control_body(vehicle, velocity_x, velocity_y, yaw_rate,altitude_rate):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000010111000111,
        0, 0, 0,
        velocity_x, velocity_y, altitude_rate,
        0, 0, 0,
        0,yaw_rate)
    vehicle.send_mavlink(msg)

def move_PID_body_manual(self,DeshHight, angl_dir, distance, time_needed):
    # Set mode to GUIDED
    self.mode = VehicleMode("GUIDED")
    roll = self.attitude.roll
    pitch = self.attitude.pitch
    
    [ angl_dir, coeff_distance ]= convert_angle_to_set_dir(self, angl_dir)
    #set_yaw_to_dir( self, angl_dir)
    set_yaw_to_dir_PID( self, angl_dir)

    distance= coeff_distance*distance
    print ( "current yaw in Deg ", math.degrees(self.attitude.yaw))
    # PID gains for yaw, X, and Y control
    # Kp_yaw = 2
    # Ki_yaw = 0.01
    # Kd_yaw = 0.01

    # Kp_vel_x = 0.9
    # Ki_vel_x = 0.0
    # Kd_vel_x = 0.0


    # Kp_vel_y = 0.9
    # Ki_vel_y = 0
    # Kd_vel_y = 0.0

    # Errors and previous errors for PID control
    error_yaw_prev = 0
    error_vel_x_prev = 0
    error_vel_y_prev = 0

    integral_yaw = 0
    integral_vel_x = 0
    integral_vel_y = 0

    Kp_yaw = 1.0
    Ki_yaw = 0.005
    Kd_yaw = 0.02

    Kp_vel_x = 3.3
    Ki_vel_x = 0.005
    Kd_vel_x = 0.01

    Kp_vel_y = 0.7
    Ki_vel_y = 0.002
    Kd_vel_y = 0.01


    # PID gains for altitude control
    Kp_alt = 1.0
    Ki_alt = 0.05
    Kd_alt = 0.2

    # Errors and previous errors for altitude PID control
    error_alt_prev = 0
    integral_alt = 0
    desired_altitude = DeshHight

    
    # Desired yaw and velocities
    desired_yaw = angl_dir # baed on the direction  needed 
    print ( "DEs yaw in Deg ", desired_yaw , " in rad",math.radians(desired_yaw) )
    desired_yaw = math.radians(desired_yaw)
    
    desired_vel_x = distance/float(time_needed)
    desired_vel_y=0.0

    print ("in NED frame Desired vx", desired_vel_x ," Des vy", desired_vel_y )
    #desired_vel_x, desired_vel_y = ned_to_body_frame(desired_vel_x, desired_vel_y, roll, pitch, math.radians(desired_yaw) )
    print ("In body frame Desired vx", desired_vel_x ," Des vy", desired_vel_y )
    
    start_time = time.time()

    send_control_body(self, desired_vel_x, desired_vel_y, 0 ,0)
    time.sleep(1) 

    velocity_current_x = (self.velocity[1])
    velocity_current_y = (self.velocity[0])
    # # velocity_current_x = velocity_in_body_frame(self)[0]
    # # velocity_current_y = velocity_in_body_frame(self)[1]

    print( "Befor PID loop vx ",velocity_current_x , "vy",velocity_current_y, "yaw", math.degrees(self.attitude.yaw) )

    while time.time() - start_time < time_needed: #not_condition_to_stop
        # Get current yaw
        yaw_current = self.attitude.yaw
        
        # Yaw error and PID control
        error_yaw = desired_yaw - yaw_current
        integral_yaw += error_yaw
        derivative_yaw = error_yaw - error_yaw_prev

        yaw_rate = yaw_current + Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw
        error_yaw_prev = error_yaw

        # Get current velocities
        velocity_current_x = (self.velocity[1])
        velocity_current_y = (self.velocity[0])
        # velocity_current_x = (velocity_in_body_frame(self)[0])
        # velocity_current_y = (velocity_in_body_frame(self)[1])
        
        print( "vx ",velocity_current_x , "vy",velocity_current_y, "yaw", math.degrees(self.attitude.yaw) )
        # X velocity error and PID control
        error_vel_x = desired_vel_x - velocity_current_x
        integral_vel_x += error_vel_x
        derivative_vel_x = error_vel_x - error_vel_x_prev

        velocity_x =  Kp_vel_x * error_vel_x + Ki_vel_x * integral_vel_x + Kd_vel_x * derivative_vel_x
        error_vel_x_prev = error_vel_x

        # Y velocity error and PID control
        error_vel_y = desired_vel_y - velocity_current_y
        integral_vel_y += error_vel_y
        derivative_vel_y = error_vel_y - error_vel_y_prev

        velocity_y =  Kp_vel_y * error_vel_y + Ki_vel_y * integral_vel_y + Kd_vel_y * derivative_vel_y
        error_vel_y_prev = error_vel_y

        # Send control to the drone
        #send_control_body(self, velocity_x, velocity_y, yaw_rate)
        velocity_current_x = (self.velocity[1])
        velocity_current_y = (self.velocity[0])
        print( "vx ",velocity_current_x , "vy",velocity_current_y, "yaw", math.degrees(self.attitude.yaw) )
        # Pause before next iteration

        # Check the current altitude
        current_altitude = self.location.global_relative_frame.alt

        # Altitude error and PID control
        error_alt = desired_altitude - current_altitude
        integral_alt += error_alt
        derivative_alt = error_alt - error_alt_prev

        altitude_rate = Kp_alt * error_alt + Ki_alt * integral_alt + Kd_alt * derivative_alt
        error_alt_prev = error_alt

        # Send control to the drone (you'll need to modify send_control_body or use a separate method to include altitude control)
        send_control_body(self, velocity_x, velocity_y, yaw_rate, altitude_rate)

        time.sleep(1)




def move_PID_body(self, angl_dir, distance, time_needed):
    # ...
    max_yaw_speed=10

    Kp_yaw = 0.5
    Ki_yaw = 0.01
    Kd_yaw = 0.05

    Kp_vel_x = 1.8
    Ki_vel_x = 0.02
    Kd_vel_x = 0.01


    Kp_vel_y = 0.25
    Ki_vel_y = 0.01
    Kd_vel_y = 0.01
   
    [ angl_dir, coeff_distance ]= convert_angle_to_set_dir(self, angl_dir)
    #set_yaw_to_dir( self, angl_dir)
    set_yaw_to_dir_PID( self, angl_dir)
    
    desired_yaw = angl_dir # baed on the direction  needed 
    print ( "DEs yaw in Deg ", desired_yaw , " in rad",math.radians(desired_yaw) )
    desired_yaw = math.radians(desired_yaw)

    distance= coeff_distance*distance
    print ( "current yaw in Deg ", math.degrees(self.attitude.yaw))

    # Need to be changed because first send control can not use this 
    desired_vel_x = distance/float(time_needed)
    desired_vel_y=0.00
    print ("in NED frame Desired vx", desired_vel_x ," Des vy", desired_vel_y )
    #desired_vel_x, desired_vel_y = ned_to_body_frame(desired_vel_x, desired_vel_y, roll, pitch, math.radians(desired_yaw) )
    print ("In body frame Desired vx", desired_vel_x ," Des vy", desired_vel_y )

    # PID controller for yaw
    pid_yaw = PID(Kp_yaw, Ki_yaw, Kd_yaw, setpoint=desired_yaw)
    pid_yaw.output_limits = (-max_yaw_speed, max_yaw_speed)  # Assuming you set a max_yaw_speed variable
    pid_yaw.sample_time = 1  # Update interval in seconds

    # PID controllers for x and y velocities
    pid_vel_x = PID(Kp_vel_x, Ki_vel_x, Kd_vel_x, setpoint=desired_vel_x)
    pid_vel_y = PID(Kp_vel_y, Ki_vel_y, Kd_vel_y, setpoint=desired_vel_y)
    
    duration= time_needed
    start_time = time.time()
    
    # Within your control loop:
    while time.time() - start_time < duration-1:
        yaw_current = self.attitude.yaw
        yaw_rate = pid_yaw(yaw_current)

        velocity_current_x = self.velocity[1]
        velocity_current_y = self.velocity[0]

        velocity_x = pid_vel_x(velocity_current_x)
        velocity_y = pid_vel_y(velocity_current_y)
   
        send_control_body(self, velocity_x, velocity_y, yaw_rate)
        velocity_current_x = (self.velocity[1])
        velocity_current_y = (self.velocity[0])
        print( "Befor PID loop vx ",velocity_current_x , "vy",velocity_current_y, "yaw", math.degrees(self.attitude.yaw) )
        time.sleep(0.1)






#time =0 means that if it was not provided move_to will use the min_time_safe 
def scan_hexagon(vehicle, drone, camera_image_width,  scan_time=0):
    a=drone.a

    drone.update_location(drone.positionX,drone.positionY)
    distance= round(math.sin(math.radians(60))* camera_image_width,2)
    
    i=0
    new_radius= a
    while new_radius > distance: 
        #first time move distance/2 to define then path of the drone 
        if i==0: 
            x=0
            y=a-distance/2 # go notrh 
            new_radius= a-(distance/2)
            print("first time go north to", y, "raduis=",new_radius )
        else: # then to go to the nex path far by distance 
            x=0
            y=-distance # move to the next path of the drone 
            print("next time go down on y by", y, "raduis=",new_radius )
        
        move_to(vehicle,x,y,scan_time)
        drone.update_location(x,y)

        x_deplacment=  round(math.sin(math.radians(60)) * new_radius,2)
        y_deplacment= round(math.cos(math.radians(60))* new_radius,2)

        # go to the vertex2 
        x= x_deplacment
        y=- y_deplacment
        print("move to v2, x",x,"y",y)
        move_to(vehicle,x,y,scan_time)
        drone.update_location(x,y)

        # go to the vertex3
        x=0
        y=-new_radius
        move_to(vehicle,x,y,scan_time)
        drone.update_location(x,y)

        # go to the vertex4
        x=-x_deplacment
        y=-y_deplacment
        move_to(vehicle,x,y,scan_time)
        drone.update_location(x,y)

        # go to the vertex5
        x=-x_deplacment
        y= y_deplacment
        move_to(vehicle,x,y,scan_time)
        drone.update_location(x,y)  

        # go to the vertex6
        x=0
        y=+new_radius
        move_to(vehicle,x,y,scan_time)
        drone.update_location(x,y)
       
        # go back to the vertex1 
        x=x_deplacment
        y=y_deplacment
        move_to(vehicle,x,y,scan_time)
        drone.update_location(x,y)

        # drone did not arrive exactly to the same vertx1 
        # correcte the error accumulated by the floating point
        # if drone.positionX!= v1_x or drone.positionY != v1_y:
        #     print("x vertex1 and y vertex1 y", drone.positionX, v1_x, drone.positionY, v1_y )
            #move_to(vehicle,drone.positionX- v1_x, drone.positionX- v1_y,time)
            #drone.update_location(v1_x-drone.positionX,v1_y-drone.positionY) #the new coordinates

        #reduce the size of hexagone and go down to new vertex1 
        new_radius= new_radius-distance
        i=i+1

    print("Come back to origin by",-(new_radius+distance +distance/2) )
    move_to(vehicle, 0, -(a-distance*i+distance/2))
    drone.update_location(0,-(a-distance*i+distance/2)) #the new coordinates


def convert_angle_to_set_dir(self, angle): 
    '''
    supppose you want the drone to have yaw -90 and move forward in respect to -90 yaw 
    and the drone currently has yaw in 90, then no need to set yaw=-90 , it is enough to 
    stay in yaw=90 and set the speed to be negative which means move bacward.

    yaw -90 and velocity positive [forward] = yaw 90 and velocity negative [bacward])

    '''
    # this function will be always called in any PID move 
    # get the current yaw 
    current_yaw = normalize_angle(math.degrees(self.attitude.yaw))
    if abs(current_yaw - ( normalize_angle(angle) -180)) < 10: # if the current angle is the opposit of what we want with error of 10 degrees 
        return [ normalize_angle(angle) -180, -1 ] # -1 is to flip the direction so go in direction of backward 
    else: 
         return [normalize_angle(angle), +1]