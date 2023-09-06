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
global vehicle
vehicle = connect(parse_connect(), wait_ready=False)
# vehicle = connect("/dev/ttyUSB0", wait_ready=False,baud=57600)
# vehicle.wait_ready(True, raise_exception=False)
signal.signal(signal.SIGINT, interrupt)


# Crash detection thresholds
angular_error_threshold = 30  # Example threshold for angular error (degrees)
acceleration_threshold = 3.0  # Example threshold for acceleration (m/s^2)

# Message handler for crash-related messages
@vehicle.on_message('CRITICAL')
def crash_message_handler(_, message):
    if 'Crash' in message.text:
        print("Crash Detected:", message.text)
        # Extract crash information from the message
        angular_error = float(message.text.split('AngErr=')[1].split('>')[0])
        acceleration = float(message.text.split('Accel=')[1].split('<')[0])

        # Crash detection logic
        if angular_error > angular_error_threshold and acceleration < acceleration_threshold:
            print("Emergency maneuver initiated to avoid crash!")

            # Perform emergency maneuver actions here
            # Adjust thrust levels, activate obstacle avoidance, etc.
            for motor in vehicle.gimbal.motors:
                motor.thrust += 0.1
            time.sleep(2)

            # Example: Decrease thrust by 20% for 1 second
            for motor in vehicle.gimbal.motors:
                motor.thrust -= 0.2

            time.sleep(1)

            #time.sleep(5)  # Allow time for the emergency maneuver
            print("Emergency maneuver complete. Resuming normal flight.")


set_a(3)
drone= Drone(0.0,0.0,1.5) # drone at the sink 
#set_home_to_zero(vehicle)
arm_and_takeoff(vehicle,drone.hight)
print( "Takeoff and wait 2 sec")
time.sleep(2)


# write_log_message(f" current_lat = {vehicle.location.global_relative_frame.lat}") 
# write_log_message(f" current_lon= {vehicle.location.global_relative_frame.lon}")

# move randomaly far from the sink
dir = 1 # go in x or east 
write_log_message(f"Go to S{dir}")
#x,y= drone.direction(dir)

time_to_pass=2 
x=3
y=0
move_to_poition(vehicle,x,y, drone.hight) # go 3m in X 
print(" move on x to" ,x , " in speed of",  x/float(time_to_pass) )
time.sleep(2)
move_to_poition(vehicle,x,y, 0) # go 3m in X 
time.sleep(8)


x=0
y=0
move_to_poition(vehicle,x,y, drone.hight) # go 3m in X 
print(" move on x to" ,x , " in speed of",  x/float(time_to_pass) )
time.sleep(10)

write_log_message(f" Coming Home")
vehicle.mode = VehicleMode ("LAND")
time.sleep(2) 

vehicle.close()
