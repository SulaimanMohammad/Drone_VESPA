import serial
import threading
import queue
import time
import re  


def find_key_by_value(dictionary, target_value):
    for key, value in dictionary.items():
        if value == target_value:
            return key
    return None 

def clear_queue_except_last(q):
    try:
        # Temporarily store the last item
        last_item = q.get_nowait()
        # Clear remaining items
        while True:
            try:
                q.get_nowait()
            except queue.Empty:
                break
        # Put the last item back
        q.put(last_item)
    except queue.Empty:
        pass

def clear_queue(q):
    while True:
        try:
            q.get_nowait()
        except queue.Empty:
            break  

# Function to send the start command
def send_start_command(ser):
    ser.write(b"START\n")  # Sending the start command


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
        
        write_log_message(f"Best_position= {best_position}")
        if best_position: 
            return best_position
        else:
            return -1

# Define a function to extract the Z component of the centroid from a string
def extract_centroid_coordinates(data):
    match = re.search(r'Cluster Centroid: \(([\d\.-]+), [\d\.-]+, ([\d\.-]+)\)', data)
    if match:
        x = float(match.group(1))  # Convert the matched X component to float
        z = float(match.group(2))  # Convert the matched Z component to float
        return x, z
    return None,None

def extract_emergency_value(data):
    try:
        # Splits the data string on spaces and extracts the numeric part.
        parts = data.split()
        if len(parts) > 1 and parts[0] == "EMERGENCY":
            # Attempt to convert the second part of the data into a float.
            return float(parts[1])
    except ValueError:
        write_log_message("Error: Unable to extract numeric value from data")
    return None


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
        write_log_message("No sensor")
        read_lidar.clear() # This flag used to exit the loop and not process reading data
        time.sleep (1)

    while read_lidar.is_set() :
        send_start_command(ser)
        emergecy_stop.clear()
        centroids_z = []
        centroids_x = []
        data_rec=0
        try:
            while True :
                if ser.in_waiting > 0:
                    data = ser.readline().decode('utf-8').rstrip()
                    if data == "Done":
                        data_rec=1 
                        break
                    if data== "Finished":
                        clear_queue(output_queue)
                        break
                    if "EMERGENCY" in data:
                        emergecy_stop.set()
                    else: 
                        x, z = extract_centroid_coordinates(data)
                        if x is not None and z is not None:
                            centroids_z.append(z)
                            centroids_x.append(x)
                else:
                    time.sleep(0.1)
        except:
            write_log_message( "Problem of reading data")

        if(data_rec==1):
            Z_to_go_distance = process_centroids(centroids_z, hostVehicle.location.global_relative_frame.alt*1000 ,ref_alt)
            if Z_to_go_distance != -1 :
                Z_to_go_distance=Z_to_go_distance / 1000.0
            else: 
                Z_to_go_distance=0
            x_distance = min(centroids_x, default=None)
            output_queue.put((Z_to_go_distance, x_distance/1000.0))
            data_ready.set() 
    if ser:
        ser.close()

def start_observer(hostVehicle, output_queue, read_lidar,emergecy_stop,data_ready, ref_alt):
    observer_thread = threading.Thread(target=observe_env, args=(hostVehicle, output_queue,read_lidar,emergecy_stop,data_ready, ref_alt))
    observer_thread.start()
    return observer_thread