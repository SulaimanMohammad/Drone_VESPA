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
    while not self.is_armable:
        print ("Waiting for self to initialise...")
        write_log_message ("Waiting for self to initialise...") 
        time.sleep(3)

    print ("Arming motors")
    write_log_message ("Arming motors")
    # Copter should arm in GUIDED mode
    self.mode    = VehicleMode("GUIDED")
    self.armed   = True

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

def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
    write_log_message (f"{get_current_function_name()} called:")
    msg = self.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b110111000111, #0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
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
            time.sleep(0.1)


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

def move_to_stages_long(self, x , y): 
    write_log_message (f"{get_current_function_name()} called:")
    # need to set speed constant and we change the time 
    # the north is the y access in the calaulation of the hex and the east is the x 
    # example reived x=a, y=b 
    # then the move to the nourth by the value of b ( y in calculation access)
    north= y  # on y
    east= x #on x 
    down=0 # stay in the same hight 
    total_time= min_time_safe()
    send_ned_velocity_stages_long(self,north/float(total_time) ,east/float(total_time), down, total_time)

def move_to_stages_short(self, x , y): 
    write_log_message (f"{get_current_function_name()} called:")
    # need to set speed constant and we change the time 
    # the north is the y access in the calaulation of the hex and the east is the x 
    # example reived x=a, y=b 
    # then the move to the nourth by the value of b ( y in calculation access)
    north= y  # on y
    east= x #on x 
    down=0 # stay in the same hight 
    total_time= min_time_safe()
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
    max_speed = 2  # Maximum speed in m/s
    longest_duration = 0  # Initialize the longest duration as 0

    for position in DIR_VECTORS:
        distance = math.sqrt(position[0]**2 + position[1]**2)  # Calculate the distance to the position
        duration = distance / max_speed *0.90 # Calculate the duration to reach the position # take only 0.90% for the safty 

        if duration > longest_duration:
            longest_duration = duration
    return int(longest_duration)

#time =0 means that if it was not provided move_to will use the min_time_safe 
def scan_hexagon(vehicle, drone, camera_image_width,  scan_time=0):
    a=drone.a
    #dave the current location 
    drone.update_location(drone.positionX,drone.positionY)
    distance= round(math.sin(math.radians(60))* camera_image_width,2)

    i=0
    # go to the main vertex1 
    move_to(vehicle,0,a,scan_time) # go to the top of hex 
    new_radius= a
    drone.update_location(0,a)
    while new_radius > distance: 
        #first time move distance/2 to define then path of the drone 
        if i==0: 
            x=0
            y=-distance/2
            new_radius= a-(distance/2)
        else: # then to go to the nex path far by distance 
            x=0
            y=-distance
        
        move_to(vehicle,x,y,scan_time)
        drone.update_location(x,y)
        time.sleep(5)
        v1_x=drone.positionX
        v1_y=drone.positionY

        x_deplacment=  round(math.sin(math.radians(60)) * new_radius,2)
        y_deplacment= round(math.cos(math.radians(60))* new_radius,2)

        # go to the vertex2 
        x= x_deplacment
        y=- y_deplacment
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
        
        # TODO drone did not arrive exactly to the same vertx1 
        # correcte the error accumulated by the floating point
        # if drone.positionX!= v1_x or drone.positionY != v1_y:
        #     print("x vertex1 and y vertex1 y", drone.positionX, v1_x, drone.positionY, v1_y )
            #move_to(vehicle,drone.positionX- v1_x, drone.positionX- v1_y,time)
            #drone.update_location(v1_x-drone.positionX,v1_y-drone.positionY) #the new coordinates

        #reduce the size of hexagone and go down to new vertex1 
        new_radius= new_radius-distance
        i=i+1

   
    #back to the start ( center of the main hexagon)
    move_to(vehicle, 0, -(a-distance*i+distance/2))
    drone.update_location(0,-(a-distance*i+distance/2)) #the new coordinates
