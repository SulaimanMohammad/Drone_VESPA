
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
import math 

#set_a(20)
def interrupt(self, signal_num, frame):
    print("Interrupted!")
    self.mode = VehicleMode ("LAND")
    time.sleep(3) 
    self.close()


signal.signal(vehicle, signal.SIGINT, interrupt)

create_log_file(os.path.dirname(os.path.abspath(__file__)),  os.path.splitext(os.path.basename(__file__))[0]) 

vehicle = connect(parse_connect(), wait_ready=False,baud=57600)
vehicle.wait_ready(True, raise_exception=False)

arm_and_takeoff(vehicle, 0.5)

write_log_message(f" current_altitude= {vehicle.location.global_relative_frame.alt}")
vehicle.mode    = VehicleMode("LOITER") #loiter mode and hover in your place 
write_log_message(f" current_altitude= {vehicle.location.global_relative_frame.alt}")
time.sleep(10)

write_log_message(f" LAND")
vehicle.mode = VehicleMode ("LAND")
time.sleep(3) 

vehicle.close()
