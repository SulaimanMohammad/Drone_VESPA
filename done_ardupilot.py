import dronekit_sitl
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import math
import time
import argparse
from expan import DIR_VECTORS

def parse_connect():
    parser=argparse.ArgumentParser (description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string = args.connect
    print ("Connection to the vehicle on %s"%connection_string)
    # Connect to the Vehicle (in this case a simulator running the same computer)
    return connection_string

def arm_and_takeoff(self, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    if self.mode.name == "INITIALISING":
        print ("initialise the self") 

    print("Basic pre-arm checks") 
    # Don't try to arm until autopilot is ready
    while not self.is_armable:
        print ("Waiting for self to initialise...")
        time.sleep(3)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    self.mode    = VehicleMode("GUIDED")
    self.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not self.armed:
        print (" Waiting for arming...")
        time.sleep(1)

    print ("Taking off starts") 
    self.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    #takeoff is asynchronous and can be interrupted if another command arrives before it reaches the target altitude so
    # check that arrived by reading the alitude relative from the ground 
    while True:
        print (" Altitude: ", self.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if self.location.global_relative_frame.alt>=aTargetAltitude*0.95:  # arrived to 95% of the altitude s
            print ("Reached target altitude")
            break
        time.sleep(1)


def calculate_relative_pos(self):
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
    print(f"Position: ({relative_x/3:.2f}, {relative_y/3:.2f}, {relative_z/3:.2f})")

def send_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):

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

    start_time = time.time()
    while time.time() - start_time < duration:
        self.send_mavlink(msg)
        time.sleep(0.1)




def send_ned_velocity_stages(self, velocity_x, velocity_y, velocity_z, duration):

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
        # for i in range(0,stage_duration):
        #     self.send_mavlink(msg)
        #     time.sleep(1)

        # Print stage information

        # # Add delay between stages if necessary
        # if stage != stages - 1:
        #     time.sleep(1)  # Add a 1-second delay between stages

# this function will calculate the speed and the time and call send_ned_velocity 
def move_to(self, x , y): 
    # need to set speed constant and we change the time 
    # the north is the y access in the calaulation of the hex and the east is the x 
    # example reived x=a, y=b 
    # then the move to the nourth by the value of b ( y in calculation access)
    north= y  # on y
    east= x #on x 
    down=0 # stay in the same hight 
    #total_time= max( abs(x)/north, abs(y)/east) #abs because the position can be - but not needed to calculatet the time  
    total_time= min_time_safe()
    print ("north", north, "east", east, "total_time",total_time)
    #print (" go north  %f , in spped of %f , and east to %f with speed of %f",north,east, north/float(total_time), east/float(total_time)  )
    send_ned_velocity(self,north/float(total_time) ,east/float(total_time), down, total_time)
    #send_ned_velocity_stages(self,north/float(total_time) ,east/float(total_time), down, total_time)
    # you can not have one speed for both and only one time for both , so we fix time and change the speed 


def move_to_stages(self, x , y): 
    # need to set speed constant and we change the time 
    # the north is the y access in the calaulation of the hex and the east is the x 
    # example reived x=a, y=b 
    # then the move to the nourth by the value of b ( y in calculation access)
    north= y  # on y
    east= x #on x 
    down=0 # stay in the same hight 
    #total_time= max( abs(x)/north, abs(y)/east) #abs because the position can be - but not needed to calculatet the time  
    total_time= min_time_safe()
    print ("north", north, "east", east, "total_time",total_time)
    #print (" go north  %f , in spped of %f , and east to %f with speed of %f",north,east, north/float(total_time), east/float(total_time)  )
    #send_ned_velocity(self,north/float(total_time) ,east/float(total_time), down, total_time)
    send_ned_velocity_stages(self,north/float(total_time) ,east/float(total_time), down, total_time)
    # you can not have one speed for both and only one time for both , so we fix time and change the speed 


def set_home_to_zero(self):

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
    max_speed = 2  # Maximum speed in m/s
    longest_duration = 0  # Initialize the longest duration as 0

    for position in DIR_VECTORS:
        distance = math.sqrt(position[0]**2 + position[1]**2)  # Calculate the distance to the position
        duration = distance / max_speed *0.90 # Calculate the duration to reach the position # take only 0.90% for the safty 

        if duration > longest_duration:
            longest_duration = duration
    return int(longest_duration)