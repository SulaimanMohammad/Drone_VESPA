
# GCS ground control station
import sys
import os
import re
import csv
import time
import threading
import signal
import struct
from datetime import datetime
import serial.tools.list_ports

parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), './src'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)

from VESPA.headers_variables import * 
from Xbee_module.xbee_usb import * 

# Shared variables
timer_count = 0
Drone_ready_lock = threading.Lock()

def find_xbee_serial_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        try:
            # Attempt to open the serial port with a Xbee 
            ser = serial.Serial(port.device, baudrate=9600, timeout=1)
            ser.close()
            # If the port opens successfully, it might be an Xbee module
            xbee_ports=port.device
        except (OSError, serial.SerialException):
            pass

    if not xbee_ports:
        print("No Xbee module found.")
        return None
    else:
        print(f"Found Xbee module(s) on port(s): {xbee_ports}")
        return xbee_ports
    
def build_emergency_message():
    message= Emergecy_header.encode()
    message += b'\n'
    return message

def decode_drone_is_ready_message(msg):
    demand_header = msg[:1].decode()
    # Extract the max_byte_count which is the next byte after the header
    max_byte_count = struct.unpack('>B', msg[1:2])[0]
    # Extract the ID using the max_byte_count
    id_start_index = 2
    id_end_index = id_start_index + max_byte_count
    id_bytes = msg[id_start_index:id_end_index]
    ids = int.from_bytes(id_bytes, 'big')
    return ids
    
def decode_data_message(message):
    # Remove header and terminal
    content = message[len(Info_header)+1:-1]
    # Split by the comma to get floats
    parts = content.split(b',')
    id_target = int(parts[0])
    sender_id = int(parts[1])
    longitude = float(parts[2])
    latitude = float(parts[3])
    data = float(parts[4])
    return id_target, sender_id, longitude, latitude, data 
       
def create_log_file():
    # Define the base log directory
    base_directory = 'mission_log'
    
    # Create the base directory if it doesn't exist
    if not os.path.exists(base_directory):
        os.makedirs(base_directory)
    
    # Create a timestamped subdirectory within the base directory
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    global target_directory
    target_directory = os.path.join(base_directory, f'target_{timestamp}')
    os.makedirs(target_directory)

def get_log_file_directory():
    return target_directory   

def write_to_csv(id, longitude, latitude, data):
    # Determine the file path
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

def GCS_listener(num_drones,Stop_flag):
    id_ready_receiced=[]
    while not Stop_flag.is_set():    
        msg= retrieve_msg_from_buffer(Stop_flag)

        if msg.startswith(Emergecy_header.encode()) and msg.endswith(b'\n'):
            os.kill(os.getpid(), signal.SIGINT) # That will call interrupt which use vehicle object to return home
        
        elif msg.startswith(Prepared_header.encode()) and msg.endswith(b'\n'):
            # New system is ready, restart the timer to wait another system to be done 
            ids=decode_drone_is_ready_message(msg)
            if ids not in id_ready_receiced:
                id_ready_receiced.append(ids)
                reset_collect_drones_ready_timer(num_drones)
            if len(id_ready_receiced)==num_drones:
                collect_drones_ready_done()

        elif msg.startswith(Info_header.encode()) and msg.endswith(b'\n'):
            id_target, sender_id, longitude,  latitude, data = decode_data_message(msg)
            if id_target==GCS_id:
                print("Drone_id ", sender_id," Longitude: ",longitude," Latitude: ", latitude, " Data: ", data)
                write_to_csv(sender_id, longitude, latitude, data)
        

def initialize_collect_drones_ready_timer(num_drones):
    reset_collect_drones_ready_timer(num_drones)
    while True:
        with Drone_ready_lock:
            global timer_count
            timer_count -= 0.1
            if timer_count <= 0:
                    break
        time.sleep(0.1)

def reset_collect_drones_ready_timer(num_drones):
    global timer_count
    with Drone_ready_lock:
        timer_count=200*num_drones # 200 is the max time given to each drone to be armable ( see  src/drone/drone_ardupilot.py )

def collect_drones_ready_done():
    global timer_count
    with Drone_ready_lock:
      timer_count=0  

def interrupt(Stop_flag):
    Stop_flag.set()
    emergency_msg= build_emergency_message()
    send_msg(emergency_msg)
    time.sleep(1)
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
    xbee_serial_port = find_xbee_serial_port()
    try: 
        global GCS_id
        GCS_id = 0 
        connect_xbee(xbee_serial_port, 9600)
    except: 
        print("Cannot connect to Xbee, reboot")
        exit(0)    

    signal.signal(signal.SIGINT, lambda sig, frame: interrupt(Stop_flag))
    create_log_file()
    GCS_launched()
    print("     GCS launched signal is sent     ")
    num_drones = int(input("\nEnter number of drones: ")) # To check that all drones are ready and armable
    global Drone_ready_lock
    Drone_ready_lock= threading.Lock() # Timer lock (timer is shared between 2 threads ( main ,GCS_listener ))
    GCS_receive_message_thread = threading.Thread(target=GCS_listener, args=(num_drones,Stop_flag))
    GCS_receive_message_thread.start()
    print("\nWaiting for signal that drones' systems are ready")
    initialize_collect_drones_ready_timer(num_drones)
    # After all the drone systems are ready, ask for start 
    wait_for_start()

    while not Stop_flag.is_set():
        time.sleep(0.1)

if __name__ == "__main__":
    main()