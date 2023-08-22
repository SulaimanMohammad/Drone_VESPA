from  expan import *
import struct
import sys
import threading
import time

multiplier = 100

def compute_checksum(data):
    """Compute the modulo-256 sum of the bytes in the data."""
    return sum(data) % 256

def build_message(self, numbers):
    # start with 'M' character
    message = self.spot["phase"].encode() #b'E' for example 
    
    # convert the float to an integer using the known multiplier
    encoded_float = int(numbers[0] * multiplier)

    if encoded_float <= 65535 : 
        message += struct.pack('>H', encoded_float)
    else:
        message += struct.pack('>I', encoded_float)
        
    # Encode the remaining numbers
    message += struct.pack('>H', numbers[1])
    message += struct.pack('>B', numbers[2])
    message += struct.pack('>H', numbers[3])
    message += struct.pack('>B', numbers[4])
    
    # Compute and append the checksum
    checksum = compute_checksum(message[1:])
    message += bytes([checksum])
    
    # End with '\n' character
    message += b'\n'
    
    return message

def decode_message(message):
    # Check start and end of the message
    if not ( message.startswith(b'E') or message.startswith(b'S') or message.startswith(b'B') ) or not message.endswith(b'\n'):
        raise ValueError("Invalid message format")
    
    # Verify the checksum
    if not compute_checksum(message[1:-2]) == message[-2]:
        raise ValueError("Invalid checksum")
    
    phase = message[0:1].decode()

    message= message[1:-1]
    
    ''' 
    Float value (2 bytes or 4 bytes, depending on its size)
    An integer (2 bytes)
    A byte (1 byte)
    Another integer (2 bytes)
    Another byte (1 byte)
    Checksum (1 byte)
    '''
     # Deduce the size of the float representation
    float_size = len(message) - 2 - 1 - 2 - 1 - 1 # Subtracting sizes of other 
    
    # check the size of first number to know how to decode 
    if float_size <= 2:
        encoded_float, = struct.unpack('>H', message[:2])
    else:
        encoded_float, = struct.unpack('>I', message[:4])
    
    first_num = encoded_float / multiplier
    
    # Decode the remaining numbers
    offset = float_size
    second_num, = struct.unpack('>H', message[offset:offset+2])
    offset += 2
    third_num = message[offset]
    offset += 1
    fourth_num, = struct.unpack('>H', message[offset:offset+2])
    offset += 2
    fifth_num = message[offset]
    
    return [phase , first_num, second_num, third_num, fourth_num, fifth_num]







class Sink_Timer:

    def __init__(sink_t, timeout=10):
        sink_t.timeout = timeout
        sink_t.last_message_time = time.time()
        sink_t.message_counter = 0  # Initialize message count
        # Start the thread that simulates message reception
        sink_t.message_thread = threading.Thread(target=sink_t.on_message_received)
        sink_t.message_thread.start()

    def simulate_message_reception(sink_t):
        #This method reception of messages
        x=0
        while x<4: # This is should be whilr True 
            time.sleep(1)
            sink_t.on_message_received()
            x+=1

    def on_message_received(sink_t):
        #This method is called whenever a message is received.
        print("Message received!")
        sink_t.last_message_time = time.time()
        sink_t.message_counter = sink_t.message_counter +1

    # here it is from the main the thread of spaning 
    def run(sink_t):
        while True:
            time.sleep(0.5)  # Sleep for a short duration to avoid busy-waiting
            elapsed_time = time.time() - sink_t.last_message_time
            if elapsed_time >= sink_t.timeout:
                sink_t.time_up()
                break

    def time_up(sink_t):
        #Called when the timer reaches its timeout without being reset.
        print("Time's up! ")
        


def spanning_sink(self):
    # initialize a timer that will be rest after reciving new message 
    app = Sink_Timer()
    app.run()
    if app.message_counter==0: #if no message received # the sink it is already irremovable 
        send_msg_drone_id= self.find_close_neigboor_2border()  # since it doesnt belong to border then find to path to border 
        if send_msg_drone_id != -1 : # there is no irremovable send msg to a drone to make it irremovable 
            # send message to a drone that had Id= send_msg_drone_id
            pass
    else: # if you already message and path is constructed then brodcast end of spanning
        # broadcast the end of spanning 
        pass


# since the 6 neighbors has 2 different regions one close to sink and one in opposit 
# check the 3 neighbors that are close to the sink to know if any is irremovable
def find_close_neigboor_2sink(self):
    neighbor_irremovable= None

    # Filter the neighbor_list for objects with "drones_in" > 0 as it contains a drone in 
    filtered_neighbors = [neighbor for neighbor in self.neighbor_list if neighbor["drones_in"] > 0]

    # Sort the neighbor_list based on the distance
    sorted_neighbors = sorted(filtered_neighbors, key=lambda x: x['distance'])

    # Extract the first 3 elements with the minimum distances
    three_min_neighbors = sorted_neighbors[:3]

    # Find the neighbor with id of the drone that is irremovable or it is sink ( distance =0 )
    neighbor_irremovable = next((neighbor for neighbor in three_min_neighbors if ( neighbor['state'] == "irremovable") ), None) # no need to check for or neighbor['distance'] == 0 because the sink is already irremovable 

    if neighbor_irremovable== None:
        return min(three_min_neighbors, key=lambda x: x["distance"])["id"] # retuen the id of the drone 
    else: 
        return -1


def find_close_neigboor_2border(self):
    neighbor_irremovable= None

    # Filter the neighbor_list for objects with "drones_in" > 0 as it contains a drone in 
    filtered_neighbors = [neighbor for neighbor in self.neighbor_list if neighbor["drones_in"] > 0]

    # Sort the neighbor_list based on the distance
    sorted_neighbors = sorted(filtered_neighbors, key=lambda x: x['distance'],reverse=True)

    # Extract the first 3 elements with the max distances ( close to the border)
    three_max_neighbors = sorted_neighbors[:3]

    # Find the neighbor with drone that is irremovable or irremovable- border because 
    # if you arrive to irremovable or irremovable-border there is no need to send message
    neighbor_irremovable = next((neighbor for neighbor in three_max_neighbors if ( neighbor['state'] == "irremovable" or neighbor['state'] == "irremovable- border"  ) ), None)

    if neighbor_irremovable== None: # if it doesnt exist check what is closest to the sink 
        return max(three_max_neighbors, key=lambda x: x["distance"])["id"] # retuen the id of the drone 
    else: 
        return -1






# each drone needs to save the irrmovable drones around so it can send messages to it as a path 
# recive all the neigboor including the state of each 
# if the current drone is irremovable then send message to the drone close to the sink 
# you can find the close one by chekcing the distance 
# the message here will include the id of the dron in subject 

# each droen should svar the id of the drone for the path to sink and for the path to border 

def spanining ( self): 
    self.check_drones_in_neigboors() # update neigboors after the finish of expansion 
                                     # here you need to safe the id of the drone that is neigboor , ask for data 

    # for the sink drone 
    if self.spot['distance']==0: # if the drone is sink ( leader of the termination of the spaning phase)
        self.spanning_sink()
    
    # for the rest of the drones 
    else: 
        if (self.state== " irremovable") or (self.state == " irremovable- border"):
            send_msg_drone_id= self.find_close_neigboor_2sink() 
            if send_msg_drone_id != -1 : # there is no irremovable send msg to a drone close to sink to make it irremovable 
                # send message to a drone that had Id= send_msg_drone_id 
                pass

        if (self.state == " irremovable" ): # it is irrmovable doesnt belong to boarder no need to check (self.state != " irremovable- border" ) because it is border no need to sed to border
            
            send_msg_drone_id= self.find_close_neigboor_2border()  # since it doesnt belong to border then find to path to border 
            if send_msg_drone_id != -1 : # there is no irremovable send msg to a drone to make it irremovable 
                # send message to a drone that had Id= send_msg_drone_id
                pass
