import dronekit_sitl
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import math
import time
import argparse
import os
import datetime
#from expan import DIR_VECTORS
import inspect
import numpy as np
from simple_pid import PID
import threading

from dronekit import Command
from pymavlink import mavutil
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

# used in the intialized pahse and done only by the drone 0 on the sink 
def new_coordinates(self, distance, bearing_deg):

    current_lat = self.location.global_relative_frame.lat
    current_lon = self.location.global_relative_frame.lon
    current_alt = self.location.global_relative_frame.alt
    # Convert latitude and longitude from degrees to radians
    lat_rad = math.radians(current_lat)
    lon_rad = math.radians(current_lon)
    
    # Convert the bearing from degrees to radians
    bearing_rad = math.radians(bearing_deg)
    
    # Earth's radius in kilometers
    R = 6371.0
    
    # Convert the distance to radians
    d_rad = distance / R
    
    # Calculate new latitude in radians
    lat2_rad = math.asin(math.sin(lat_rad) * math.cos(d_rad) + 
                         math.cos(lat_rad) * math.sin(d_rad) * math.cos(bearing_rad))
    
    # Calculate new longitude in radians
    lon2_rad = lon_rad + math.atan2(math.sin(bearing_rad) * math.sin(d_rad) * math.cos(lat_rad), 
                                    math.cos(d_rad) - math.sin(lat_rad) * math.sin(lat2_rad))
    
    # Convert the new latitude and longitude from radians back to degrees
    lat2 = math.degrees(lat2_rad)
    lon2 = math.degrees(lon2_rad)
    
    return lat2, lon2 

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
            self.flush()
            time.sleep(0.1)

def move_to_poition(self, x , y, z) :
    write_log_message (f"{get_current_function_name()} called:")
    north= y  # on y
    east= x #on x 
    down= -z # stay in the same hight 
    send_ned_position(self, north, east, down)

# this function will calculate the speed and the time and call send_ned_velocity 
def move_to_speed(self, x , y, time_passed=0): 
    write_log_message (f"{get_current_function_name()} called:")
    # need to set speed constant and we change the time 
    # the north is the y access in the calaulation of the hex and the east is the x 
    # example reived x=a, y=b 
    # then the move to the nourth by the value of b ( y in calculation access)
    north= y  # on y
    east= x #on x 
    down=0 # stay in the same hight 
    if time_passed==0:
        # total_time= min_time_safe()
        total_time= 10
    else:
        total_time=time_passed
    send_ned_velocity(self, north/float(total_time), east/float(total_time), down, total_time)
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
        # total_time= min_time_safe()
        total_time= 10
    else:
        total_time=time_passed
    send_ned_velocity_stages_long(self,north/float(total_time) ,east/float(total_time), down, total_time)

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

def set_altitude(self, target_altitude):
    # Create a location object with the same latitude and longitude as the current location, but with the desired altitude
    target_location = LocationGlobalRelative(self.location.global_relative_frame.lat,
                                             self.location.global_relative_frame.lon,
                                             target_altitude)
    # Command the vehicle to go to the target location
    self.simple_goto(target_location)


'''
yaw values are fluctuating around 0 and 365 degrees, which is very close to the north direction (0 degrees) 
but represented differently.
This is a common issue in dealing with angles, as 0 degrees and 365 degrees are the same direction
but on opposite sides of the compass.

You can handle this by normalizing the yaw angle to a value between 0 and 360 degrees 
and then checking if it's close to 0 degrees.

'''

# Get the current position to hold
#hold_position = self.location.global_relative_frame
#self.simple_goto(hold_position)

 
def normalize_roll(roll_rad):
    return math.atan2(math.sin(roll_rad), math.cos(roll_rad))

def normalize_pitch(pitch_rad):
    return max(min(pitch_rad, math.pi/2), -math.pi/2)

def normalize_yaw(yaw_rad):
    return math.atan2(math.sin(yaw_rad), math.cos(yaw_rad))

def normalize_angle(angle):
    while angle < 0:
        angle += 360
    while angle >= 360:
        angle -= 360
    return angle

def set_data_rate(self, rate_hz=15):
    # Request velocity/position message updates
    msg_rate = self.message_factory.request_data_stream_encode(
        target_system=0,  # target_system
        target_component=0,  # target_component
        req_stream_id=6,  # MAV_DATA_STREAM enum value for POSITION including velocity 
        req_message_rate=rate_hz,  # Rate in Hz
        start_stop=1  # 1 to start sending, 0 to stop
    )

    self.send_mavlink(msg_rate)    


new_yaw_data = threading.Event()
def yaw_listener(self, name, message):
    global yaw_rad
    global last_event_time_yaw
    global interval_between_events_yaw
    # Record current time
    current_time = time.time()
    # If this is not the first event, print the time difference
    if last_event_time_yaw is not None:
        interval_between_events_yaw = current_time - last_event_time_yaw
        #print(f"YAW Time between two events: {interval_between_events_yaw} seconds")

    # Update the last event time
    last_event_time_yaw = current_time

    # Signal that new data is available
    new_yaw_data.set()
    # Extract yaw from the message (in radians)
    yaw_rad = message.yaw

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
        #print(f"Time between two events: {interval_between_events} seconds")

    # Update the last event time
    last_event_time = current_time

    # Signal that new data is available
    velocity_components = value
    # Convert string values to float
    velocity_listener = [float(v) for v in velocity_components]
    new_velocity_data.set()

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
    self.flush()

# Send a command to control velocity and 
def send_control_body(self, velocity_x, velocity_y, altitude_rate):
    msg = self.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # MAV_FRAME_LOCAL_NED
        0b010111000111,
        0, 0, 0,
        velocity_x, velocity_y, altitude_rate,
        0, 0, 0,
        0, 0)
    self.send_mavlink(msg)
    self.flush()

def set_yaw_to_dir_PID(self, target_yaw, relative=True, max_yaw_speed=20):
    
    global yaw_rad
    yaw_rad = normalize_angle(math.degrees(self.attitude.yaw))
    global interval_between_events_yaw
    interval_between_events_yaw=0
    global last_event_time_yaw
    last_event_time_yaw=None

    kp= abs(yaw_rad - normalize_angle(target_yaw) )/190.0  #1.1
    ki=0.02
    kd=0.008
    print(kp)

    # Target values
    target_altitude = self.location.global_relative_frame.alt
    target_latitude = self.location.global_relative_frame.lat
    target_longitude = self.location.global_relative_frame.lon

    pid = PID(kp, ki, kd, setpoint=target_yaw)
    pid.output_limits = (-max_yaw_speed, max_yaw_speed)  # Limits to ensure the output is within valid bounds
    
    self.add_message_listener('ATTITUDE', yaw_listener)

    while True:
        new_yaw_data.wait()
        
        current_yaw = normalize_angle(math.degrees(yaw_rad))
        error = normalize_angle(target_yaw - current_yaw)

        # Correcting error if it's shorter to turn the other way
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # Check if error is within tolerance
        if abs(error) < 1.5:
            break
        # Get the PID output
        yaw_speed = pid(error)
        
        # Determine the direction (clockwise or counterclockwise)
        direction = 1 if error >= 0 else -1

        # Send the yaw command
        set_yaw_PID(self, abs(error), abs(yaw_speed), direction, relative)

        time.sleep(interval_between_events_yaw)
        new_yaw_data.clear()

    self.remove_message_listener('ATTITUDE', yaw_listener)
    print("Yaw set to:", target_yaw)
    self.mode    = VehicleMode("LOITER") #loiter mode and hover in your place 
    time.sleep(1.5)
    self.mode     = VehicleMode("GUIDED")


def ned_to_body(self,velocity_vec):
    # Retrieve the vehicle's roll, pitch, and yaw angles
    roll = normalize_roll(self.attitude.roll)
    pitch = normalize_pitch(self.attitude.pitch)
    #yaw = normalize_yaw(self.attitude.yaw)
    yaw= math.radians( normalize_angle(math.degrees(self.attitude.yaw)))

    print( "        yaw", math.degrees(yaw), "pitch", math.degrees(pitch), "roll", math.degrees(roll))

    Vx_ned=velocity_vec[0]
    Vy_ned=velocity_vec[1]
    Vz_ned=velocity_vec[2]

    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    cos_pitch = np.cos(pitch)
    sin_pitch = np.sin(pitch)
    cos_roll = np.cos(roll)
    sin_roll = np.sin(roll)

    # Optimized Rotation matrix from NED to Body frame
    R_ned_to_body = np.array([
        [cos_yaw * cos_pitch, sin_yaw * cos_pitch, -sin_pitch],
        [cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll, sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll, cos_pitch * sin_roll],
        [cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll, sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll, cos_pitch * cos_roll]])
        
    # Create the velocity vector in NED frame
    V_ned = np.array([Vx_ned, Vy_ned, Vz_ned])
    
    # Transform the velocity to the body frame
    V_body = np.dot(R_ned_to_body, V_ned)
    
    return V_body


def move_body_PID(self,DeshHight, angl_dir, distance,max_velocity=2): #max_velocity=2
    # Set mode to GUIDED
    if self.mode != "GUIDED":
        self.mode = VehicleMode("GUIDED")

    roll = self.attitude.roll
    pitch = self.attitude.pitch

    global velocity_listener
    velocity_listener=0
    global interval_between_events
    interval_between_events=0
    global last_event_time
    last_event_time=None

    
    [ angl_dir, velocity_direction ]= convert_angle_to_set_dir(self, angl_dir)
    set_yaw_to_dir_PID( self, angl_dir)

    # PID gains for yaw, X, and Y control
    Kp_yaw = 0.25
    Ki_yaw = 0.03
    Kd_yaw = 0.05

    Kp_vel_x = 0.9
    Ki_vel_x = 0.02
    Kd_vel_x = 0.01

    Kp_vel_y = 0.1
    Ki_vel_y = 0.02
    Kd_vel_y = 0.01


    # PID gains for altitude control
    Kp_vel_z = 0.6
    Ki_vel_z = 0.03
    Kd_vel_z = 0.01

    # Errors and previous errors for PID control
    error_vel_x_prev = 0
    error_vel_y_prev = 0

    integral_vel_x = 0
    integral_vel_y = 0

    # Errors and previous errors for altitude PID control
    error_alt_prev = 0
    integral_vel_z = 0
    desired_vel_z = 0

    # Desired yaw and velocities
    desired_yaw = normalize_angle(angl_dir) # baed on the direction  needed 
    print ( "DEs yaw in Deg ", desired_yaw , " in rad",math.radians(desired_yaw) )
    

    
    # PID controller for yaw
    max_yaw_speed=5
    pid_yaw = PID(Kp_yaw, Ki_yaw, Kd_yaw, setpoint=desired_yaw)
    pid_yaw.output_limits = (-max_yaw_speed, max_yaw_speed)  # Assuming you set a max_yaw_speed variable
    pid_yaw.sample_time = 1 # Update interval in seconds


    desired_vel_x = velocity_direction* max_velocity
    desired_vel_y=0.0
    desired_vel_z=0.0

    self.add_attribute_listener('velocity', on_velocity)
    
    start_time = time.time()

    send_control_body(self, desired_vel_x, desired_vel_y, 0)
   
    previous_velocity_x=0 
    remaining_distance= distance 
    velocity_current_x=0
    k=0.08
    start_control_timer=0
    acc=True
    while remaining_distance >= 0.1:
        new_velocity_data.wait()
        
        print( "---------------------------------------------------------------------")
        if remaining_distance < distance*( 1-0.6):
            #desired_vel_x= desired_vel_x*( 1-0.2)
            reduction= k*velocity_current_x*remaining_distance 
            desired_vel_x=desired_vel_x-reduction
            acc=False
            desired_vel_x = max(desired_vel_x, 0.1)

        # # Get current yaw
        yaw_current = self.attitude.yaw
        current_yaw = normalize_angle(math.degrees(yaw_current))
        error = (desired_yaw - current_yaw)
        error = normalize_angle(desired_yaw - current_yaw)

        # Correcting error if it's shorter to turn the other way
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # Get the PID output
        yaw_speed = pid_yaw(error)
        
        # Determine the direction (clockwise or counterclockwise)
        direction_yaw = 1 if error >= 0 else -1

        # Send the yaw command
        set_yaw_PID(self, abs(error), abs(yaw_speed), direction_yaw)

        # Get current velocities from NED frame to body 
        velocity_body   =ned_to_body(self,velocity_listener )
        velocity_current_x=(velocity_body[0])
        velocity_current_y=(velocity_body[1])
        velocity_current_z=(velocity_body[2])


        # X velocity error and PID control
        error_vel_x = desired_vel_x - velocity_current_x
        integral_vel_x += error_vel_x
        derivative_vel_x = error_vel_x - error_vel_x_prev

        velocity_x = velocity_current_x+ Kp_vel_x * error_vel_x + Ki_vel_x * integral_vel_x + Kd_vel_x * derivative_vel_x
        

        # Y velocity error and PID control
        error_vel_y = desired_vel_y - velocity_current_y
        integral_vel_y += error_vel_y
        derivative_vel_y = error_vel_y - error_vel_y_prev

        velocity_y = velocity_current_y + Kp_vel_y * error_vel_y + Ki_vel_y * integral_vel_y + Kd_vel_y * derivative_vel_y
        error_vel_y_prev = error_vel_y

        # Altitude error and PID control
        error_vel_z = desired_vel_z - velocity_current_z
        integral_vel_z += error_vel_z
        derivative_vel_z = error_vel_z - error_alt_prev

        altitude_rate = velocity_current_z +Kp_vel_z * error_vel_z + Ki_vel_z * integral_vel_z + Kd_vel_z * derivative_vel_z
        error_alt_prev = error_vel_z
        
        # Check the current altitude
        current_altitude = self.location.global_relative_frame.alt
        
        
        if (time.time() - start_control_timer > 0.5) or (abs(error_vel_x) > abs(error_vel_x_prev) and acc==True ) or (abs(error_vel_x) < abs(error_vel_x_prev) and acc==False ) or abs(velocity_y -desired_vel_y) > 0.1 or abs(altitude_rate- desired_vel_z)> 0.5:
            # Send control to the drone 
            print("     controle        ")
            print( "velocity_x", velocity_x, "velocity_y",velocity_y)
            send_control_body(self, velocity_x, velocity_y, altitude_rate)
            start_control_timer= time.time()
        
        remaining_distance= remaining_distance - (float(interval_between_events* abs( (velocity_current_x + previous_velocity_x)/2.0 )))
        
        print( "vx ",velocity_current_x , "vy",velocity_current_y, "vz",velocity_current_z ,"yaw error= ",error, "current alt= ", current_altitude  )
        print( "time", time.time() - start_time , "distance left : ",remaining_distance, "dis speed", desired_vel_x )
        
        # save data for the next iteration 
        error_vel_x_prev = error_vel_x
        previous_velocity_x= velocity_current_x # save the current for next iteration 
        
        # Clear the event so we can wait for the next update
        new_velocity_data.clear()
    
    send_control_body(self, 0, 0, 0) # stop 
    self.remove_attribute_listener('velocity', on_velocity)

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
    if abs(current_yaw - ( normalize_angle(angle) -180)) < 20: # if the current angle is the opposit of what we want with error of 10 degrees 
        return [ normalize_angle(angle) -180, -1 ] # -1 is to flip the direction so go in direction of backward 
    else: 
         return [normalize_angle(angle), +1]
    