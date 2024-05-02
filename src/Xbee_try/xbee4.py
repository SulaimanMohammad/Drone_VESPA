import sys
import os
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
from Xbee_module.xbee_usb import connect_xbee,close_xbee_port
from VESPA.VESPA_module import *
from VESPA.expansion import expansion_listener,first_exapnsion
from VESPA.spanning import spanning
# Serial port settings for XBee
serial_port = '/dev/ttyUSB2'  # Default UART port on Raspberry Pi
baud_rate = 9600  # Baud rate for XBee
connect_xbee(serial_port, baud_rate) 
drone = Drone(2,0.0,0.0,1)
create_log_file() 
vehicle = connect ("127.0.0.1:14580", wait_ready=False) # for simulation 
#arm_and_takeoff(vehicle,drone.drone_alt)
set_data_rate(vehicle, 20)

sq3=1.73
a=13
# x=(sq3 / 2.0) * a
# y=(3.0 / 2.0) * a
print("drone.ref_alt",drone.ref_alt)
print("rone.drone_alt",drone.drone_alt)
time.sleep(3)       
try:
    while True:
        first_exapnsion(drone,vehicle)
        print( "call spannng")
        vehicle.mode     = VehicleMode("RTL")
        break
        #spanning(drone)
        time.sleep(100)  # Interval for sending messages
except KeyboardInterrupt:
    print("Interrupt received, stopping...")
    running = False  # Signal the reading thread to stop
finally:
    # Close the serial connection
    close_xbee_port()
    #vehicle.close()
    print("Serial connection closed.")

