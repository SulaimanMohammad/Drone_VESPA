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
from pathlib import Path
import queue
from dronekit import Command
from pymavlink import mavutil
import sys 
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '../Lidar/RPI/'))
sys.path.append(parent_directory)
from lidar import *

#-------------------------------------------------------------
#------------------  Lidar Functions -------------------------
#-------------------------------------------------------------
def avoidance_emergency(self, lidar_queue,emergecy_stop, ref_alt, data_ready):
    # If an object found in the emergency zone ( very close object to the drone)
    if emergecy_stop.is_set():
        safe_zone= 1
        check_objects_time=0
        goal_altitude=0
        actual_alt=get_altitude(self)
        if (actual_alt+safe_zone)<(ref_alt*2) : # Not close to the max then go up  
            v_z= -0.5 #"go up "
            target_alt= (actual_alt+1)
        elif  (actual_alt-safe_zone)>(ref_alt-2) :# Not close to the min alt then go down 
            v_z= +0.5 #"go down
            target_alt= (actual_alt-safe_zone)

        # Wait until the current altitude arrives to target_alt 
        while abs(get_altitude(self) - target_alt)>0.5: 
            send_control_body(self, 0, 0, v_z)
            time.sleep(0.2)
            
        send_control_body(self, 0, 0, 0)
        time.sleep(0.2)
        send_control_body(self, 0, 0, 0)
        time.sleep(0.2)

        # Clear flags 
        emergecy_stop.clear()
        data_ready.clear() 
        # Wait until an emergency or a objects data in case of no emergency 
        while ( (not data_ready.is_set()) and (not emergecy_stop.is_set()) ):
            time.sleep(0.1)
        # If no emergency and objects found then find velcoity to avoid them 
        if data_ready.is_set(): 
            output_read=full_scan_avoidence(self, lidar_queue,data_ready )
            if not output_read==None: 
                velocity_z, Z_to_go_distance, min_x_close_object, check_objects_time, goal_altitude=output_read    
        # another emergency (close object) then change the alt again and wait untile clear env 
        else: 
            output_read = avoidance_emergency(self, lidar_queue,emergecy_stop, ref_alt,data_ready) 
            if not output_read==None: 
                velocity_z, Z_to_go_distance, min_x_close_object, check_objects_time, goal_altitude=output_read   
        return [velocity_z, Z_to_go_distance, min_x_close_object, check_objects_time, goal_altitude]
    else:
        return None 
    
def full_scan_avoidence(self, lidar_queue,data_ready ):
    #scan and find all avilable objects and corresponding veloxity on z to avoid it  
    # check_objects_time is used check when the drone arrived to the alt ( in case parometer was not accurate)
    velocity_z=0
    check_objects_time=0 
    goal_altitude= get_altitude(self)
    min_x_close_object= None
    if( data_ready.is_set() ):
        try: 
            Z_to_go_distance, min_x_close_object = lidar_queue.get(timeout=1)  # Adjust timeout as needed
            data_ready.clear()
            if (Z_to_go_distance and Z_to_go_distance != 0 ):
                check_objects_time= time.time()
                if( Z_to_go_distance>0):
                    velocity_z=-0.5 # 0.5 m/s 
                else: 
                    velocity_z=+0.5 # go down  
                goal_altitude= get_altitude(self) + Z_to_go_distance
        except:
            write_log_message("No data received from observer")
            velocity_z=0      
    return [velocity_z, Z_to_go_distance, min_x_close_object ,check_objects_time, goal_altitude]        


def scan_befor_movement(self,lidar_queue,data_ready,emergecy_stop,ref_alt):    
    # wait until data came back "emergency or data of objects"
    while ( (not data_ready.is_set()) and (not emergecy_stop.is_set()) ):
        time.sleep(0.1)

    if not emergecy_stop.is_set():     
        output_read = full_scan_avoidence(self, lidar_queue,data_ready )
    else:
        output_read = avoidance_emergency(self, lidar_queue,emergecy_stop, ref_alt, data_ready) 
    
    if not output_read==None: 
        velocity_z,Z_to_go_distance,min_x_close_object, check_objects_time, goal_altitude=output_read 
    # Returned data will be used to send signal to drone to move 
    return velocity_z, Z_to_go_distance, min_x_close_object, check_objects_time, goal_altitude

def regular_scan(self,lidar_queue,data_ready,emergecy_stop,ref_alt,velocity_z, min_x_close_object, check_objects_time,goal_altitude,Z_to_go_distance):
    # Check emergency first 
    output_read= avoidance_emergency(self, lidar_queue,emergecy_stop, ref_alt, data_ready)     
    if not output_read==None: 
        velocity_z, Z_to_go_distance, min_x_close_object, check_objects_time, goal_altitude=output_read      
    
    # If no emergency and data is ready and the drone is not in process of changing altitude from previous scan 
    if( check_objects_time ==0 and abs( goal_altitude-get_altitude(self))<0.2 and data_ready.is_set() ):
        output_read= full_scan_avoidence(self, lidar_queue,data_ready )
        if not output_read==None: 
            velocity_z, Z_to_go_distance, min_x_close_object, check_objects_time, goal_altitude=output_read
    
    # If the drone arrived to alitude for avoiding object from previous scan then reset vars to start another scan 
    if ( check_objects_time>0 and (abs( goal_altitude- get_altitude(self))<0.2 or time.time()-check_objects_time>=abs(Z_to_go_distance*1.2) ) ):  # distance traveled t=d/0.5= d*0.5
        min_x_close_object= None
        Z_to_go_distance=0
        check_objects_time=0
        velocity_z=0
    
    return velocity_z, Z_to_go_distance, min_x_close_object, check_objects_time, goal_altitude
#-------------------------------------------------------------
#-------------------------------------------------------------

#-------------------------------------------------------------
#-------------- Connection and logs Functions ----------------
#-------------------------------------------------------------
# Declare global variables for logs 
filename = " "

def create_log_file(log_dir_name="mission_log", script_name="mission"):
    base_dir=os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
    # Get the current timestamp
    global timestamp 
    timestamp = datetime.datetime.now().strftime("%H-%M|%Y-%m-%d")

    # Create the directory path for the new log
    global log_directory
    log_directory = os.path.join(base_dir, log_dir_name, timestamp)
    os.makedirs(log_directory, exist_ok=True)
    global filename
    filename= "travel.log"
    log_file_path = os.path.join(log_directory, filename)

def get_log_file_directory():
    return log_directory

def open_log_file():
    # Get the absolute path of the log file
    log_file_path = os.path.join(log_directory, filename)
    # Open the log file
    return open(log_file_path, 'a') # dont use "w" because it remove all what was before after each open

def write_log_message(message):
    with open_log_file() as log_file:
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        log_file.write(f"{timestamp}: {message}\n")
        global print_show
        if print_show =="True": 
            print(message)

def get_current_function_name():
    # Get the frame of the calling function
    frame = inspect.currentframe().f_back
    # Get the name of the calling function from the frame
    function_name = frame.f_code.co_name
    return function_name

def parse_connect(): 
    parser=argparse.ArgumentParser (description='commands')
    parser.add_argument('--connect', type=str, help="Connection address in the format IP:PORT")
    parser.add_argument('command', nargs='?', default="", help="Command to process (telemetry or empty)")
    args, _ = parser.parse_known_args()
    # Connect to the Vehicle (in this case a simulator running the same computer)
    return args


def drone_connect():
    telemetry1_baudrate,telemetry2_baudrate=get_connection_paramter()
    global simulation_dont_hover
    simulation_dont_hover= False
    
    # Parse the arguments
    retuned_parse = parse_connect()
    # Check the argument
    if retuned_parse.connect:
        print("Simulation")
        vehicle = connect (retuned_parse.connect, wait_ready=False) # for simulation 
        simulation_dont_hover= True
    
    elif retuned_parse.command == "telemetry":
        print("Telemetry connection")
        vehicle = connect("/dev/ttyUSB0", baud= telemetry1_baudrate,  wait_ready=False, rate=10) #for telemetry 1
    elif retuned_parse.command == "":
        print("Actual drone connection")
        vehicle = connect("/dev/serial0", baud= telemetry2_baudrate,  wait_ready=False) # for raspberry pi
    else:
        print(f"Unknown argument: {retuned_parse.command}")
        sys.exit(0)
    
    vehicle.wait_ready(True, raise_exception=False)
    return vehicle


def get_connection_paramter():
    # Current script directory
    current_dir = Path(__file__).resolve().parent
    # Path to the parent directory
    parent_dir = current_dir.parent
    global Operational_Data_path
    Operational_Data_path = parent_dir/'Operational_Data.txt'
    
    with open(Operational_Data_path, 'r') as file:
        for line in file:
            # Check if line contains max_acceleration
            if "telemetry1_baudrate" in line:
                telemetry1_baudrate = int(line.split('=')[1].strip())
            # Check if line contains max_deceleration
            elif "telemetry2_baudrate" in line:
                telemetry2_baudrate = int(line.split('=')[1].strip())
            elif "print_show" in line:
                global print_show
                print_show = str(line.split('=')[1].strip())                
    return telemetry1_baudrate,telemetry2_baudrate 


#-------------------------------------------------------------
#-------------------------------------------------------------


def check_gps_fix(self):
    while not self.is_armable:
        write_log_message("Waiting for the vehicle to become armable...")
        time.sleep(1)
    # Get the GPS fix type
    fix_type = self.gps_0.fix_type
    # Get the number of satellites
    num_satellites = self.gps_0.satellites_visible
    if fix_type >= 3 and num_satellites >= 8:
        return True
    else: 
        return False
            
def check_armablity(self):
    start_wait_ToArm=time.time() 
    while not self.is_armable and (time.time()- start_wait_ToArm < 200):
            write_log_message ("Waiting for self to initialise and Armability...")
            time.sleep(1)
    if not self.is_armable:
        write_log_message ("Could not arm the drone")
        raise Exception("Vehicle not armable")
    else: 
        fix_type = self.gps_0.fix_type
        num_satellites = self.gps_0.satellites_visible
        if fix_type >= 3 and num_satellites >= 8:
            return True
        else:
            write_log_message ("GPS is not ready")   
            return False

def arm_and_takeoff(self, aTargetAltitude):
    write_log_message (f"{get_current_function_name()} called:") 
    
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    if self.mode.name == "INITIALISING":
        write_log_message ("Initialising the drone") 

    write_log_message("Basic pre-arm checks") 
    # TODO do not try to take off if the alituduid is not zero 
    # Don't try to arm until autopilot is ready
    # TODO check that gps lock before the fly by listening to the raw data of the channel
    #GPs usually it is done by the system  of pixhawc 
    # TODO check that the battery is enough to fly
    
    check_armablity(self)

    write_log_message ("Arming motors")
    # Copter should arm in GUIDED mode
    self.mode    = VehicleMode("GUIDED")
    self.armed   = True
    write_log_message(f"Battery: {self.battery}")

    # Confirm vehicle armed before attempting to take off
    while not self.armed:
        write_log_message (" Waiting for arming...")
        time.sleep(1)

    write_log_message ("Taking off starts")
    self.simple_takeoff(aTargetAltitude) # Take off to target altitude
    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    #takeoff is asynchronous and can be interrupted if another command arrives before it reaches the target altitude so
    # check that arrived by reading the alitude relative from the ground 
    while True:
        write_log_message (f" Altitude: {get_altitude(self)}")
        #Break and return from function just below target altitude.
        if get_altitude(self)>=aTargetAltitude*0.95:  # arrived to 95% of the altitude s
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
        current_alt = get_altitude(self)

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
def new_coordinates(initial_longitude, initial_latitude,distance, bearing_deg):
    EARTH_RADIUS = 6371000  # Approximate value for the Earth's radius

  # Convert angle from degrees to radians
    angle_radians = math.radians(bearing_deg)

    # Convert latitude and longitude from degrees to radians
    initial_longitude = math.radians(initial_longitude)
    initial_latitude = math.radians(initial_latitude)

    # Calculate the new latitude using the Haversine formula
    new_latitude = math.asin(
        math.sin(initial_latitude) * math.cos(distance / EARTH_RADIUS) +
        math.cos(initial_latitude) * math.sin(distance / EARTH_RADIUS) * math.cos(angle_radians)
    )

    # Calculate the new longitude using the Haversine formula
    new_longitude = initial_longitude + math.atan2(
        math.sin(angle_radians) * math.sin(distance / EARTH_RADIUS) * math.cos(initial_latitude),
        math.cos(distance / EARTH_RADIUS) - math.sin(initial_latitude) * math.sin(new_latitude)
    )

    # Convert the new latitude and longitude back to degrees
    new_latitude = math.degrees(new_latitude)
    new_longitude = math.degrees(new_longitude)

    return new_longitude, new_latitude

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

def check_mode(self):
    # Set mode to GUIDED
    if self.mode.name != "GUIDED":
        self.mode = VehicleMode("GUIDED")
    
def get_acceleration():
    # Current script directory
    current_dir = Path(__file__).resolve().parent
    # Path to the parent directory
    parent_dir = current_dir.parent
    global Operational_Data_path
    Operational_Data_path = parent_dir/'Operational_Data.txt'
    
    with open(Operational_Data_path, 'r') as file:
        for line in file:
            # Check if line contains max_acceleration
            if "max_acceleration" in line:
                max_acceleration = float(line.split('=')[1].strip())
            # Check if line contains max_deceleration
            elif "max_deceleration" in line:
                max_deceleration = float(line.split('=')[1].strip())
    # Output the extracted values
    write_log_message(f" Read Max Acceleration: {max_acceleration}")
    write_log_message(f"Read Max Deceleration: {max_deceleration}")
    
    return max_acceleration,max_deceleration 

def get_lidar_setting():
    # Current script directory
    current_dir = Path(__file__).resolve().parent
    # Path to the parent directory
    parent_dir = current_dir.parent
    global Operational_Data_path
    Operational_Data_path = parent_dir/'Operational_Data.txt'
    lidar_scan= False
    with open(Operational_Data_path, 'r') as file:
        for line in file:
            # Check if line contains max_acceleration
            if "lidar_scan" in line:
                lidar_scan_val =(line.split('=')[1].strip())
                if lidar_scan_val == "True":
                    lidar_scan = True
                else:
                    lidar_scan = False         
    # Output the extracted values
    write_log_message(f"lidar_scan: {lidar_scan}")
    return lidar_scan 

def save_acceleration(max_acceleration, max_deceleration): 
    # Open the file in write mode and write the new values
    new_content = []
    with open(Operational_Data_path, 'r') as file:
        for line in file:
            if line.startswith('max_acceleration'):
                new_content.append(f'max_acceleration = {max_acceleration}\n')
            elif line.startswith('max_deceleration'):
                new_content.append(f'max_deceleration = {max_deceleration}\n')
            else:
                new_content.append(line)

    # Write the updated content back to the file
    with open(Operational_Data_path, 'w') as file:
        file.writelines(new_content)
        

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

def set_yaw_to_dir_PID(self, target_yaw, relative=True, max_yaw_speed=10):
    
    global yaw_rad
    yaw_rad = normalize_angle(math.degrees(self.attitude.yaw))
    global interval_between_events_yaw
    interval_between_events_yaw=0
    global last_event_time_yaw
    last_event_time_yaw=None

    kp= (abs(yaw_rad - normalize_angle(target_yaw) )/180.0)/2  #1.1
    ki=0.02
    kd=0.02

    # Target values
    target_altitude = get_altitude(self)
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
    write_log_message(f"Yaw set to:{target_yaw}")
    wait_and_hover(self, 1.5)

def ned_to_body(self,velocity_vec):
    # Retrieve the vehicle's roll, pitch, and yaw angles
    roll = normalize_roll(self.attitude.roll)
    pitch = normalize_pitch(self.attitude.pitch)
    #yaw = normalize_yaw(self.attitude.yaw)
    yaw= math.radians( normalize_angle(math.degrees(self.attitude.yaw)))

    #print( "        yaw", math.degrees(yaw), "pitch", math.degrees(pitch), "roll", math.degrees(roll))
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

def get_desired_speed(current_speed, remaining_distance, max_acceleration, max_deceleration, max_speed,safety_margin=0.9):
    max_deceleration=abs(max_deceleration)
    # Deceleration phase calculations with safety margin
    decel_time = max_speed / (max_deceleration * safety_margin)
    decel_distance = max_speed * decel_time - 0.5 * (max_deceleration * safety_margin) * decel_time**2
    # Check if we should start decelerating
    if remaining_distance <= decel_distance:
        # If within deceleration distance, calculate desired speed for smooth stop
        desired_speed = (1 * max_deceleration * remaining_distance)**0.5
    else:
        # If not within deceleration distance, aim for max speed
        # Optionally, scale the desired speed based on remaining distance
        scaling_factor = min(1, remaining_distance / (decel_distance * 2))  # Example scaling factor, adjust as needed
        desired_speed = max_speed*scaling_factor
    
    return round(desired_speed,1)
    
def calculate_acceleration(v1_y, v2_y, delta_t):
    return (v2_y - v1_y) / delta_t

def exponential_smoothing(estimated_acceleration, a_observed, alpha):
    return alpha * a_observed + (1 - alpha) * estimated_acceleration

def update_acceleration_estimate(estimated_acceleration, current_velocity, previous_velocity_x, dt):
    alpha = 0.5  # Smoothing factor
    if dt: 
        a_observed = calculate_acceleration(previous_velocity_x, current_velocity, dt)
        estimated_acceleration = exponential_smoothing(estimated_acceleration, a_observed, alpha)
        
    return estimated_acceleration

def hold_yaw_PID(self, desired_yaw):
    # PID controller for yaw
    Kp_yaw = 0.25
    Ki_yaw = 0.03
    Kd_yaw = 0.05
    max_yaw_speed=5
    pid_yaw = PID(Kp_yaw, Ki_yaw, Kd_yaw, setpoint=desired_yaw)
    pid_yaw.output_limits = (-max_yaw_speed, max_yaw_speed)  # Assuming you set a max_yaw_speed variable
    pid_yaw.sample_time = 1 # Update interval in seconds

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
    write_log_message(f"yaw error= {error}" )

def velocity_PID(desired_vel_x,desired_vel_z, velocity_body_vector):
    # PID gains for yaw, X, and Y control
    Kp_vel_x = 1.2
    Ki_vel_x = 0.02
    Kd_vel_x = 0.01

    Kp_vel_y = 0.1
    Ki_vel_y = 0.02
    Kd_vel_y = 0.01

    Kp_vel_z = 0.6
    Ki_vel_z = 0.03
    Kd_vel_z = 0.01

    # Errors and previous errors for PID control
    error_vel_x_prev = 0
    error_vel_y_prev = 0
    error_alt_prev = 0

    integral_vel_x = 0
    integral_vel_y = 0
    integral_vel_z = 0

    desired_vel_y=0.0

    velocity_current_x=(velocity_body_vector[0])
    velocity_current_y=(velocity_body_vector[1])
    velocity_current_z=(velocity_body_vector[2])
    
    # X velocity error and PID control
    error_vel_x = desired_vel_x - velocity_current_x
    integral_vel_x += error_vel_x
    derivative_vel_x = error_vel_x - error_vel_x_prev

    velocity_x = velocity_current_x+ Kp_vel_x * error_vel_x + Ki_vel_x * integral_vel_x + Kd_vel_x * derivative_vel_x
    # save data for the next iteration 
    error_vel_x_prev = error_vel_x

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

    velocity_z = velocity_current_z +Kp_vel_z * error_vel_z + Ki_vel_z * integral_vel_z + Kd_vel_z * derivative_vel_z
    error_alt_prev = error_vel_z

    return velocity_x, velocity_y, velocity_z

def move_body_PID(self, angl_dir, distance, emergency_message_flag ,ref_alt=9.7, max_acceleration=0.5, max_deceleration= -0.5 , max_velocity=2): #max_velocity=2
     
    global velocity_listener
    velocity_listener=0
    global interval_between_events
    interval_between_events=0
    global last_event_time
    last_event_time=None

    previous_velocity_x=0 
    remaining_distance= distance 
    velocity_current_x=0
    start_control_timer=0
    PID_time=0
    previous_desired_vel_x=0
    estimated_acceleration=1

    check_mode(self) 
    max_acceleration,max_deceleration= get_acceleration()
    lidar_scan=get_lidar_setting() 
    set_yaw_to_dir_PID( self, angl_dir)
    
    # Desired yaw and velocities
    desired_vel_x = get_desired_speed(0, distance,max_acceleration, max_deceleration, max_velocity) 
    desired_vel_y=0
    desired_vel_z = 0
    desired_yaw = normalize_angle(angl_dir) # baed on the direction  needed 

    self.add_attribute_listener('velocity', on_velocity)
    
    some_velocity_threshold=0.08
    velocity_updated=False
    
    #-------------------------------------------------------------
    #--------------- Launch Lidar for avoiding objects------------
    #-------------------------------------------------------------
    if lidar_scan:  
        lidar_queue = queue.Queue()
        read_lidar= threading.Event() 
        read_lidar.set()
        emergecy_stop= threading.Event() 
        data_ready= threading.Event() 
        check_objects_time=0 
        min_x_close_object= None 
        velocity_z_lidar=0
        old_velocity_z= 0
        observer_thread = start_observer(self, lidar_queue, read_lidar, emergecy_stop,data_ready, ref_alt)
        time.sleep (3) # Wait 3 second to be sure that the Lidar is connected 
        if (not read_lidar.is_set()): # senor is not avilable then dont move 
            send_control_body(self, 0, 0, 0)
            self.remove_attribute_listener('velocity', on_velocity)
            raise Exception("No Lidar found")
        try:
            velocity_z_lidar,Z_to_go_distance, min_x_close_object, check_objects_time, goal_altitude= scan_befor_movement(self,lidar_queue,data_ready,emergecy_stop,ref_alt)
        except:
            write_log_message("First scan is failed")
            velocity_z_lidar=0 
        desired_vel_z= velocity_z_lidar
    #-------------------------------------------------------------
    #-------------------------------------------------------------

    start_time = time.time()
    send_control_body(self, desired_vel_x, desired_vel_y, desired_vel_z)     
    
    while remaining_distance >= 0.1 and (not emergency_message_flag.is_set()):
        old_velocity_z= desired_vel_z # Set initial in case lidar is not activated
        #-------------------------------------------------------------
        #--------------------- Lidar for regular_scan-----------------
        #-------------------------------------------------------------
        if lidar_scan: 
            old_velocity_z= velocity_z_lidar
            velocity_z_lidar, Z_to_go_distance, min_x_close_object, check_objects_time, goal_altitude= regular_scan(self,lidar_queue,data_ready,emergecy_stop,ref_alt,velocity_z_lidar, min_x_close_object, check_objects_time,goal_altitude,Z_to_go_distance)
            desired_vel_z=velocity_z_lidar
            # Send the segnal and dont wait for the next round of PID 
            if(old_velocity_z != velocity_z_lidar and velocity_z_lidar==0 ):
                send_control_body(self, velocity_x, velocity_y, velocity_z)
        #-------------------------------------------------------------
        
        new_velocity_data.wait()
        
        # Get current velocities from NED frame to body 
        velocity_body   =ned_to_body(self,velocity_listener )
        velocity_current_x=(velocity_body[0])
        velocity_current_y=(velocity_body[1])
        velocity_current_z=(velocity_body[2])

        # Separate conditions for updating desired_vel_x during acceleration and deceleration
        is_accelerating = previous_velocity_x < velocity_current_x 
        is_decelerating = previous_velocity_x > velocity_current_x 
        
        # Additional condition: Only update desired_vel_x if current velocity is close enough to the previous desired_vel_x
        is_close_to_desired = abs(velocity_current_x - desired_vel_x) < some_velocity_threshold
        
        # Update desired_vel_x based on conditions
        if is_close_to_desired:
            previous_desired_vel_x = desired_vel_x
            desired_vel_x = get_desired_speed(velocity_current_x, remaining_distance, max_acceleration, max_deceleration, max_velocity)            
            velocity_updated=True
    
        time_elaps = time.time() - PID_time
        if is_accelerating and velocity_current_x >= desired_vel_x or (is_decelerating and velocity_current_x <= desired_vel_x) or (velocity_updated or time_elaps > (desired_vel_x/estimated_acceleration)/2) or (old_velocity_z !=desired_vel_z ): # (old_velocity_z !=desired_vel_z ) in case the lidar found object and new movement on Z 
            PID_time=time.time()
            velocity_x, velocity_y, velocity_z= velocity_PID(desired_vel_x,desired_vel_z, velocity_body)
            hold_yaw_PID(self, desired_yaw)
            velocity_updated=False
        else:
            velocity_x= desired_vel_x
            velocity_y=0
            velocity_z=desired_vel_z

        if previous_desired_vel_x < desired_vel_x and velocity_current_x < desired_vel_x and previous_velocity_x <= velocity_current_x:
            #Calcul ACC
            max_acceleration= update_acceleration_estimate(max_acceleration, velocity_current_x, previous_velocity_x, interval_between_events)
            estimated_acceleration= max_acceleration
        elif previous_desired_vel_x > desired_vel_x and velocity_current_x > desired_vel_x and previous_velocity_x >= velocity_current_x: 
            #Calcul DEAC
            max_deceleration= update_acceleration_estimate(max_deceleration, velocity_current_x, previous_velocity_x, interval_between_events)
            estimated_acceleration= abs(max_deceleration) #Need to be used to detmin when PID will be called
        
        #-------------------------------------------------------------
        #------Lidar for reduce velocity close ti the objects --------
        #-------------------------------------------------------------
        # reduce the velocity on X to avoid any collision and give time to increase altitude  
        if lidar_scan: 
            if(min_x_close_object != None and min_x_close_object < remaining_distance):
                velocity_x=desired_vel_x*0.5
        #-------------------------------------------------------------

        
        # Send control to the drone reguraly 
        if (time.time() - start_control_timer > 0.2):
            send_control_body(self, velocity_x, velocity_y, velocity_z)
            start_control_timer= time.time()
        
        remaining_distance= remaining_distance - (float(interval_between_events* abs( (velocity_current_x + previous_velocity_x)/2.0 )))
        # save the current for next iteration to calculate the traveld distance 
        previous_velocity_x= velocity_current_x 
        
        write_log_message( f"\nvx= {velocity_current_x} , vy= {velocity_current_y} ,vz= {velocity_current_z}, current alt= {get_altitude(self)}" )
        write_log_message( f"time= {time.time() - start_time}, distance left= {remaining_distance}, desired_vel_x speed= {desired_vel_x} \n" )
        write_log_message( "---------------------------------------------------------------------")

        # Clear the event so we can wait for the next update
        new_velocity_data.clear()

    # Arrive to destination stop  
    send_control_body(self, 0, 0, 0)
    save_acceleration(max_acceleration, max_deceleration)
    self.remove_attribute_listener('velocity', on_velocity)
    time.sleep(5)

    
def go_to_ref_altitude(self,ref_alt=9.7):
    time.sleep(0.5) # stablize Z 

    if( get_altitude(self) > ref_alt * 0.95 ): # ref_alt is under , go downe 
        desired_vel_Z= 0.5
    elif( get_altitude(self) < ref_alt * 0.95 ): 
        desired_vel_Z=-0.5
    else: 
        desired_vel_Z=0 

    if desired_vel_Z!=0: 
        start_control_timer=0 
        while ( abs(get_altitude(self) - ref_alt)>=0.2 and  (get_altitude(self) > ref_alt-1 or get_altitude(self) < ref_alt+1 )  ):
            if (time.time() - start_control_timer > 0.2):
                write_log_message(f"alt= {get_altitude(self)}, velocity_z= {desired_vel_Z}" )
                send_control_body(self, 0, 0, desired_vel_Z)
                start_control_timer= time.time()

        # Ensure stop 
        send_control_body(self, 0, 0, 0)
        time.sleep(0.2)
        send_control_body(self, 0, 0, 0)
    
    time.sleep(1)


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
    
def wait_and_hover(self, time_req):
    global simulation_dont_hover
    if not simulation_dont_hover:
        self.mode    = VehicleMode("LOITER") #loiter mode and hover in your place 
    time.sleep(time_req)
    self.mode     = VehicleMode("GUIDED")

def hover(self):
    global simulation_dont_hover
    if not simulation_dont_hover:
        self.mode    = VehicleMode("LOITER") #loiter mode and hover in your place 

def set_to_move(self):
    self.mode     = VehicleMode("GUIDED")

def get_altitude(self):
    return self.location.global_relative_frame.alt


def search_for_sink_tag(slef):
    pass 