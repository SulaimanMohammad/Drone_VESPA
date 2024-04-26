import serial
import threading
import queue
import time
import re  

# Function to send the start command
def send_start_command(ser):
    ser.write(b"START\n")  # Sending the start command

def find_key_by_value(dictionary, target_value):
    for key, value in dictionary.items():
        if value == target_value:
            return key
    return None 

def process_centroids(centroids_z, current_alt, ref_alt):
    max_altitude= (ref_alt *2) *1000
    safe_zone= (ref_alt-2)*1000
    D = 1*1000
    # Check if the drone is securety distance D from all object, Drone ditance is 0 relative to all the objects
    all_objects_sufficiently_distant = all((z + D <= 0) if z < 0 else (z - D >= 0) for z in centroids_z)

    # Determine the best position for the Drone based on the check
    best_position = -1
    if all_objects_sufficiently_distant:
        best_position=0
    else:
        movements={} # values of moveing up or down from each object
        for z in centroids_z:
            movements[z+D]=z
            movements[z-D]=z
        accepted={} # values of movement that preserve the security distance from all objects
        for mov,value in movements.items():
            match=True
            for z in centroids_z:
                if abs(z-mov)<D:
                    match=False
            if match and mov not in accepted:
                accepted[mov]=value

        #  The calculations now take in account the altitude of the drone ( relative to the ground)
        alt_ground ={} # Contains the drone altitude for each accepted movement
        for x in accepted.keys():
            alt_ground[x]=current_alt+x

        altitude_accepted=[] # Chose the altitude that keep the drone between safe_zone and max_altitude
        for x,value in alt_ground.items():
            if value> safe_zone and value< max_altitude:
                altitude_accepted.append(value)

        if altitude_accepted: # find the min altitude_accepted to travel as minimum a possible
            min_altitude_accepted= min(altitude_accepted,key=abs)
            best_position= find_key_by_value(alt_ground, min_altitude_accepted) # return the movement that is required to achive the best altitude to avoid objects
        
        print("best_position", best_position)
        if best_position: 
            return best_position
        else:
            return -1
        
def extract_centroid_coordinates(data):
    match = re.search(r'Cluster Centroid: \(([\d\.-]+), [\d\.-]+, ([\d\.-]+)\)', data)
    if match:
        print("x",match.group(1),"z", match.group(2))
        x = float(match.group(1))  # Convert the matched X component to float
        z = float(match.group(2))  # Convert the matched Z component to float
        return x, z
    return None,None

def serial_listener(ser, dataQueue, stop_event):
    while not stop_event.is_set():  # Check if the stop event is signaled
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').rstrip()
            dataQueue.put(data)
        else:
            time.sleep(0.1)

def observe_env(hostVehicle, output_queue, read_lidar,emergecy_stop,data_ready, ref_alt):
    ser = None
    try: 
        ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    except: 
        print("No sensor")
        read_lidar.clear()
        time.sleep (1)
    if ser:
        ser.close()

def start_observer(hostVehicle, output_queue, read_lidar,emergecy_stop,data_ready, ref_alt):
    observer_thread = threading.Thread(target=observe_env, args=(hostVehicle, output_queue,read_lidar,emergecy_stop,data_ready, ref_alt))
    observer_thread.start()
    return observer_thread