from dronekit import connect, VehicleMode, mavutil
import time

# Connect to the Vehicle
vehicle = connect("/dev/ttyACM0", baud=115200, wait_ready=True) # USB connection 
#vehicle = connect("/dev/serial0", baud= 921600,  wait_ready=False,rate=10) # for raspberry p
#vehicle = connect("/dev/ttyUSB0", baud= 57600,  wait_ready=False, rate=10) #for telemetry
vehicle.wait_ready(True, raise_exception=False) 

#Capture STATUSTEXT Messages
def handle_statustext_callback(self, name, message):
    #print("Received STATUSTEXT:", message.text)
    # TODO should be modified for all types of Pixhawk6C not only 6C
    if 'Pixhawk6C' in message.text:  
        print("ID:", message.text)

# Listener for STATUSTEXT messages
vehicle.add_message_listener('STATUSTEXT', handle_statustext_callback)

# Send the param_fetch_all command, without this no information would be sent 
vehicle._master.param_fetch_all()

# Allow some time for the Pixhawk to respond and for the callback to capture the messages
time.sleep(5)

# Stop Listener
vehicle.remove_message_listener('STATUSTEXT', handle_statustext_callback)
vehicle.close()
