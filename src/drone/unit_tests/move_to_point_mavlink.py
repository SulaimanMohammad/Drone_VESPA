import os
import datetime
import sys
import signal

# Get the parent directory path
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
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
# vehicle = connect (parse_connect(), wait_ready=False) # for simulation 
vehicle = connect("/dev/serial0", baud= 921600,  wait_ready=False,rate=10) # for raspberry p
#vehicle = connect("/dev/ttyUSB0", baud= 57600,  wait_ready=False, rate=10) #for telemetry
vehicle.wait_ready(True, raise_exception=False) # for raspberry pi & telemetry only 
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

            print("Emergency maneuver complete. Resuming normal flight.")


drone_hight=2 
arm_and_takeoff(vehicle,drone_hight)
print( "Takeoff and wait 2 sec")
time.sleep(2)

time_to_pass=2 
x=3
y=0
move_to_poition(vehicle,x,y, drone_hight) # go 3m in X 
print(" move on x to" ,x , " in speed of",  x/float(time_to_pass) )
time.sleep(2)
move_to_poition(vehicle,x,y, 0) # go 3m in X 
time.sleep(8)


x=0
y=0
move_to_poition(vehicle,x,y, drone_hight) # go 3m in X 
print(" move on x to" ,x , " in speed of",  x/float(time_to_pass) )
time.sleep(10)

write_log_message(f" Coming Home")
vehicle.mode = VehicleMode ("LAND")
time.sleep(2) 

vehicle.close()
