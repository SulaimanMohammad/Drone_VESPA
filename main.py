
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

def wait_start_signal(self):
    # Dummy flag class with an is_set method needed for retrieve_msg_from_buffer
    class waitflag:
        def is_set():
            return False
    ready_msg_broadcasted=[]
    start_received=True  
    while start_received: 
        msg= retrieve_msg_from_buffer(waitflag)
        
        if msg.startswith(Emergecy_header.encode()) and msg.endswith(b'\n'):
            self.emergency_stop()
            break

        if msg.startswith(Prepared_header.encode()) and msg.endswith(b'\n'):
            ids=self.decode_drone_is_ready_message(msg)
            if ids not in ready_msg_broadcasted:
                send_msg(msg)
                ready_msg_broadcasted.append(ids)
    
        if msg.startswith(Inauguration_header.encode()) and msg.endswith(b'\n'):
            write_log_message("Start VESPA receivd ")
            send_msg(msg) # Reforward the message to the other drones in case they are far from GCS 
            start_received=False

def wait_start_GCS():
    # Dummy flag class with an is_set method needed for retrieve_msg_from_buffer
    class waitflag:
        def is_set():
            return False 
    GCS_started_recived=True  
    while GCS_started_recived: 
        msg= retrieve_msg_from_buffer(waitflag)
        if msg.startswith(GCS_Started_header.encode()) and msg.endswith(b'\n'):
            # Re-brodcast the message so it arrives to the drones far from the GCS local machine 
            send_msg(GCS_Started_header.encode()+ b'\n') # GCS has started notify the fleet to start preparation
            GCS_started_recived=False

def main():
    # Create drone object and also connect to Xbee (so any data arrives will be captured in the buffer )
    drone = Drone(0.0,0.0,0.0, parse_arguments())
    vehicle=drone_connect()
    set_data_rate(vehicle, 20)
    signal.signal(signal.SIGINT, lambda sig, frame: drone.interrupt(vehicle))
    # Wait until GCS is launched 
    wait_start_GCS()
    create_log_file() # Create log of the process
    drone.check_VESPA_safety(vehicle) # check if a reboot occurred due to a sudden error and deal wit it as emergency 
    # Configure parameter of drone based on VESPA
    config_parameters(vehicle, drone)
    if check_armablity(vehicle):
        drone.system_is_ready() # Send message to GCS that system is ready and armable 
    wait_start_signal(drone) # Wait the start flag to initiate VESPA

    try:
        first_exapnsion(drone, vehicle)
        spanning(drone,vehicle)
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