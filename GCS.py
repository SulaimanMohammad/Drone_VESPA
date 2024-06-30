
# GCS ground control station

import sys
import os

parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), './src'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), './src'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)

from VESPA.VESPA_module import *
from VESPA.expansion import first_exapnsion,further_expansion

def interrupt(drone):
    emergency_msg= drone.build_emergency_message()
    send_msg(emergency_msg)    
    revert_file(file_path, changes)
    close_xbee_port()
    print("Serial connection closed.")
    exit(0)


def main():    
    # Create drone object of VESPA 
    drone = Drone(0,0.0,0.0)
    signal.signal(signal.SIGINT, lambda sig, frame: interrupt(drone))
    try:
        while True:
            time.sleep(0.5)
    except SystemExit:
        pass
if __name__ == "__main__":
    main()
