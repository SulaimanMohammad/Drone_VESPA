
import sys
import os
import argparse
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), './src'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
from VESPA.VESPA_module import *
from VESPA.expansion import first_exapnsion,further_expansion
from VESPA.spanning import spanning
from VESPA.balancing import balancing


parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), './src'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
from drone.set_drone_parameters import * 

def parse_arguments():
    # Check if Id is provided instead of fixed one in opertional_data that needs many RPI 
    parser = argparse.ArgumentParser(description='Drone ID Parser')
    parser.add_argument('--id', type=int, help='ID of the drone')
    # Parse known args to allow for additional arguments to be passed through
    args, _ = parser.parse_known_args()
    if args.id != None:
        return args.id
    else: 
        return None

def wait_start_signal():
    # Dummy flag class with an is_set method needed for retrieve_msg_from_buffer
    class waitflag:
        def is_set():
            return False
    
    start_recived=True  
    while start_recived: 
        msg= retrieve_msg_from_buffer(waitflag)
        if msg.startswith(Inauguration_header.encode()) and msg.endswith(b'\n'):
            write_log_message("Start VESPA receivd ")
            send_msg(Inauguration_header.encode()+ b'\n') 
            start_recived=False
    time.sleep(2)

def main():
    
    # Create vehicle object 
    logs= create_log_file() 
    vehicle=drone_connect()
    set_data_rate(vehicle, 20)
    # Create drone object of VESPA 
    drone = Drone(0.0,0.0,0.0, parse_arguments())
    # Configure parameter of drone based on VESPA
    config_parameters(vehicle, drone)
    signal.signal(signal.SIGINT, lambda sig, frame: drone.interrupt(vehicle))

    wait_start_signal() # Wait the start flag to initiate VESPA

    try:
        first_exapnsion(drone, vehicle)
        spanning(drone)
        balancing(drone, vehicle)

        while drone.check_termination():
            further_expansion( drone, vehicle)
            spanning(drone,vehicle)
            # Since the irremovable drone as stuck in spanning, then it will retuen and try to performe balancing 
            if drone.state==Irremovable and drone.VESPA_termination.is_set():
                break
            # NO need to do same for border because VESPA_termination will be set in balancing and then while evaluation comes
            balancing(drone, vehicle )

    except:
        write_log_message("Error in performing VESPA")
        drone.emergency_stop()
    finally:
        drone.return_home(vehicle) 
        close_xbee_port()
        vehicle.close()
        write_log_message("Serial connection closed.")

if __name__ == "__main__":
    main()