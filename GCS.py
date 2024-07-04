
# GCS ground control station
import sys
import os
import re

parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), './src'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)

# Make the sation Xbee worrk with USB serial 
def modify_file(file_path):
    # Read the file contents
    with open(file_path, 'r') as file:
        content = file.read()
    # Find the original Tx and Rx values using regular expressions
    tx_match = re.search(r'Tx=(\d+)', content)
    rx_match = re.search(r'Rx=(\d+)', content)
    if tx_match and rx_match:
        original_tx = tx_match.group(0)
        original_rx = rx_match.group(0)
        # Create the changes dictionary
        changes = {
            original_tx: 'Tx=None',
            original_rx: 'Rx=None'
        }
        # Apply the changes
        for old, new in changes.items():
            content = content.replace(old, new)
        
        # Write the modified content back to the file
        with open(file_path, 'w') as file:
            file.write(content)
        
        print("File modified successfully.")
        return changes
    else:
        print("Tx or Rx values not found in the file.")
        return None

def revert_file(file_path, changes):
    if changes is None:
        print("No changes to revert.")
        return
    # Read the file contents
    with open(file_path, 'r') as file:
        content = file.read()
    
    # Revert the changes
    for new, old in changes.items():
        print(new, old)
        content = content.replace(old, new)
    
    # Write the reverted content back to the file
    with open(file_path, 'w') as file:
        file.write(content)
    print("File reverted successfully.")

# Define the file path
file_path = 'src/Operational_Data.txt'

# Call the function to modify the file
changes = modify_file(file_path)

from VESPA.VESPA_module import *
from VESPA.expansion import first_exapnsion,further_expansion
from drone.set_drone_parameters import * 

def interrupt(drone):
    emergency_msg= drone.build_emergency_message()
    send_msg(emergency_msg)    
    revert_file(file_path, changes)
    close_xbee_port()
    print("Serial connection closed.")
    exit(0)

def wait_for_start():
    message = "When you are ready to start VESPA press enter:"
    input(message)  # This will display the message and wait for the user to press Enter
    msg= Inauguration_header.encode()+ b'\n'
    print(msg)
    send_msg(msg)
    print("Proceeding...")

def main():    
    # Create drone object of VESPA 
    drone = Drone(0,0.0,0.0)
    signal.signal(signal.SIGINT, lambda sig, frame: interrupt(drone))
    wait_for_start()
    try:
        while True:
            time.sleep(0.5)
    except SystemExit:
        pass
if __name__ == "__main__":
    main()
