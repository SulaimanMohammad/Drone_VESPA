import serial
import threading
import queue
import time
import re  # Import the regular expression module

# Function to send the start command
def send_start_command(ser):
    print("Sending START command to ESP...")
    ser.write(b"START\n")  # Sending the start command


def process_centroids(centroids_z):
    print("centroids_z",centroids_z)
    D = 50
    # Correctly applying the logic based on the explanation
    all_objects_sufficiently_distant = all((z + D <= 0) if z < 0 else (z - D >= 0) for z in centroids_z)

    # Determine the best position for the Raspberry Pi based on the check
    best_position = -1
    if all_objects_sufficiently_distant:
        best_position=0
    else:
        movements={}
        for z in centroids_z:
            movements[z+D]=z
            movements[z-D]=z

        #print("movements",movements)

        accepted={}
        for mov,value in movements.items():
            #print(mov,value)
            match=True
            for z in centroids_z:
                if abs(z-mov)<D:
                    match=False
            if match and mov not in accepted:
                accepted[mov]=value

        #print("accepted",accepted)

        if accepted.keys():
#            best_position= min(accepted.keys())
             best_position= min(accepted.keys(),key=abs)

    #print("best_position",best_position)
    return best_position
# Define a function to extract the Z component of the centroid from a string
def extract_centroid_coordinates(data):
    print(data) 
#    match = re.search(r'Cluster Centroid: \((?:[\d\.-]+), (?:[\d\.-]+), ([\d\.-]+)\)', data)
    match = re.search(r'Cluster Centroid: \(([\d\.-]+), [\d\.-]+, ([\d\.-]+)\)', data)

    #print(match)
    if match:
        print(match.group(1), match.group(2))
        x = float(match.group(1))  # Convert the matched X component to float
        z = float(match.group(2))  # Convert the matched Z component to float
        return x, z
    return None,None

# Define a function to run in a separate thread for listening to serial port
def serial_listener(port, baud_rate,dataQueue):
    with serial.Serial(port, baud_rate, timeout=1) as ser:
        send_start_command(ser)
        while True:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').rstrip()
                dataQueue.put(data)  # Put the received data into the queue


def observe_env(): 
    # Create a thread-safe queue
    dataQueue = queue.Queue()
    centroids_z = []  # List to store Z components of centroids
    centroids_x = []  # List to store Z components of centroids
    # Start the serial listener thread
    listener_thread = threading.Thread(target=serial_listener, args=('/dev/ttyUSB0', 115200, dataQueue))
    listener_thread.daemon = True  # This thread dies when the main thread dies
    listener_thread.start()

    try:
    #    send_start_command()  # Send the command to ESP to start its operation
        while True:
            # Process data if available
            if not dataQueue.empty():
                received_data = dataQueue.get()
                if received_data == "Done":
                    Z_to_go_ditance= process_centroids(centroids_z)  # Call the processing function
                    x_distance= min(centroids_x)
                    return Z_to_go_ditance, x_distance
                    # centroids_z.clear()  # Reset the list for potential future data sets
                    # centroids_x.clear()
                else:
                    x, z = extract_centroid_coordinates(received_data)  # Extract both coordinates
                    if x is not None and z is not None:
                        centroids_z.append(z)  # Append Z component to list
                        centroids_x.append(x)
                    #z = extract_centroid_z(received_data)
                    #if z is not None:
                    #    centroids_z.append(z)
                        #print(f"Z component added: {z}")
            else:
                time.sleep(0.1)  # Short sleep to avoid maxing out CPU when there's no data
    except KeyboardInterrupt:
        print("Program terminated!")
