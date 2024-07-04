
# GCS ground control station
import sys
import os
import re
import csv

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
        
        return changes
    else:
        return None

def revert_file(file_path, changes):
    if changes is None:
        return
    # Read the file contents
    with open(file_path, 'r') as file:
        content = file.read()
    
    # Revert the changes
    for new, old in changes.items():
        content = content.replace(old, new)
    
    # Write the reverted content back to the file
    with open(file_path, 'w') as file:
        file.write(content)

# Define the file path
file_path = 'src/Operational_Data.txt'

# Call the function to modify the file
changes = modify_file(file_path)

from VESPA.VESPA_module import *
from VESPA.expansion import first_exapnsion,further_expansion
from drone.set_drone_parameters import * 

def write_to_csv(id, longitude, latitude,data):
    file_path = os.path.join(get_log_file_directory(), 'target.csv')
    # Check if the file already exists and read its contents
    data_exists = False
    if os.path.isfile(file_path):
        with open(file_path, mode='r', newline='', encoding='utf-8') as file:
            reader = csv.reader(file)
            for row in reader:
                if row[0] == str(id):
                    data_exists = True
                    break
    # Write data if the id is not found
    if not data_exists:
        with open(file_path, mode='a', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            # If the file was newly created, write the header
            if file.tell() == 0:
                writer.writerow(['id', 'longitude', 'latitude', 'data'])
            writer.writerow([id, longitude, latitude, data])

def GCS_listener(self,Stop_flag):
    while not Stop_flag.is_set():    
        msg= retrieve_msg_from_buffer(Stop_flag)

        if msg.startswith(Emergecy_header.encode()) and msg.endswith(b'\n'):
            os.kill(os.getpid(), signal.SIGINT) # That will call interrupt which use vehicle object to return home
        
        elif msg.startswith(Prepared_header.encode()) and msg.endswith(b'\n'):
            # New system is ready, restart the timer to wait another system to be done 
            reset_collect_drones_ready_timer(self)

        elif msg.startswith(Info_header.encode()) and msg.endswith(b'\n'):
            id_target, sender_id, longitude,  latitude, data = self.decode_data_message(msg)
            if id_target==self.id:
                print("Drone_id ", sender_id," Longitude: ",longitude," Latitude: ", latitude, " Data: ", data)
                write_to_csv(sender_id, longitude, latitude, data)
        

def initialize_collect_drones_ready_timer(self):
    reset_collect_drones_ready_timer(self)
    while True:
        with self.Drone_ready_lock:
            self.timer_count -= 0.1
            if self.timer_count <= 0:
                    break
        time.sleep(0.1)

def reset_collect_drones_ready_timer(self):
    with self.Drone_ready_lock:
        self.timer_count=20 
    
def interrupt(drone,Stop_flag):
    emergency_msg= drone.build_emergency_message()
    send_msg(emergency_msg)
    Stop_flag.set()
    time.sleep(10)
    revert_file(file_path, changes)
    close_xbee_port()
    print("Serial connection closed.")
    sys.exit(0)

def wait_for_start():
    message = "All drones' systems are ready, When you are ready start VESPA by pressing enter"
    input(message)  # This will display the message and wait for the user to press Enter
    msg= Inauguration_header.encode()+ b'\n'
    print(msg)
    send_msg(msg)
    print("Proceeding...")

def GCS_launched():
    send_msg(GCS_Started_header.encode()+ b'\n') # GCS has started notify the fleet to start preparation
    time.sleep(0.5)

def main():
    Stop_flag= threading.Event()  
    logs= create_log_file() 
    # Create drone object of VESPA 
    drone = Drone(0,0.0,0.0)
    signal.signal(signal.SIGINT, lambda sig, frame: interrupt(drone,Stop_flag))
    GCS_launched()
    # Create GCS_listener to receive messages
    GCS_receive_message_thread = threading.Thread(target=GCS_listener, args=(drone,Stop_flag))
    GCS_receive_message_thread.start()
    drone.Drone_ready_lock= threading.Lock() # Timer lock (timer is shared between 2 threads ( main ,GCS_listener ))
    initialize_collect_drones_ready_timer(drone)
    # After all the drone systems are ready, ask for start 
    wait_for_start()

    while not Stop_flag.is_set():
        time.sleep(0.5)

if __name__ == "__main__":
    main()
