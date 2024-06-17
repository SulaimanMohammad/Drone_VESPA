import sys
import os

parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), './src'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
from Xbee_module.xbee_pigpio import connect_xbee,close_xbee_port
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), './src'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)

from VESPA.VESPA_module import *
from VESPA.expansion import first_exapnsion,further_expansion

parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), './src'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
from drone.set_drone_parameters import * 

def main():
    # Create vehicle object 
    logs= create_log_file() 
    vehicle = connect (parse_connect(), wait_ready=False) # for simulation 
    #vehicle = connect("/dev/serial0", baud= 921600,  wait_ready=False) # for raspberry pi
    #vehicle = connect("/dev/ttyUSB0", baud= 57600,  wait_ready=False, rate=10) #for telemetry 
    vehicle.wait_ready(True, raise_exception=False) 
    set_data_rate(vehicle, 20)

    # Serial port settings for XBee
    #serial_port = '/dev/ttyUSB0'  # Default UART port on Raspberry Pi
    #baud_rate = 9600  # Baud rate for XBee
    #connect_xbee(serial_port, baud_rate) 

    # Create drone object of VESPA 
    drone = Drone(0,0.0,0.0)
    print("id=", drone.id)
    # Configure parameter of drone based on VESPA
    #config_parameters(vehicle, drone)
    signal.signal(signal.SIGINT, lambda sig, frame: drone.interrupt(vehicle))

    first_exapnsion(drone, vehicle)


if __name__ == "__main__":
    main()
# drone.return_home(vehicle) 
# close_xbee_port()
# vehicle.close()
# print("Serial connection closed.")
