from  expan import *
import struct
import sys
import threading
import time


def compute_checksum(data):
    """Compute the modulo-256 sum of the bytes in the data."""
    return sum(data) % 256

def build_message(self, numbers):
    # start with 'M' character
    message = self.phase.encode() #b'E' for example 
    
    # convert the float to an integer using the known multiplier
    encoded_float = int(numbers[0] * multiplier) # multiplier = 100 from expan 

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
    def __init__(sink_t,self,timeout=10):
        sink_t.timeout = timeout
        sink_t.message_counter = 0  # Initialize message count
        sink_t.remaining_time = sink_t.timeout
        sink_t.lock_sink = threading.Lock()  # Create a lock
        sink_t.message_thread = threading.Thread(target=sink_listener, args=(sink_t,self,timeout,))
        sink_t.message_thread.start()
        sink_t.end_of_spanning= threading.Event()

    def run(sink_t, self):
        while True:
            with sink_t.lock_sink:  # Acquire the lock
                sink_t.remaining_time -= 0.5
                print(f"Remaining time: {sink_t.remaining_time:.2f} seconds")
                if sink_t.remaining_time <= 0:
                    sink_t.time_up(sink_t,self)
                    break
            time.sleep(0.5)

    def time_up(sink_t,self):
        #Called when the timer reaches its timeout without being reset.
        print("Time's up! ")
        
        if sink_t.message_counter==0: #if no message received # the sink it is already irremovable 
            send_msg_drone_id= self.find_close_neigboor_2border()  # since it doesnt belong to border then find to path to border 
            if send_msg_drone_id != -1 : # there is no irremovable send msg to a drone to make it irremovable 
                # send message to a drone that had Id= send_msg_drone_id
                pass
        else: # if you already have  message and path is constructed then brodcast end of spanning 
            # broadcast the end of spanning 
            # Here to brodcast the end of the pahse 
            # the message contains -127 as id to confirm that is end of pahse   
            #   after the time is up ' waiting time '
            # the message here should contains a ref to sink when it is arrive to border, it should come back to the sink 
            sink_t.end_of_spanning.set() # flag to tell listener that it is the end 
            sink_listener.join() # stop listenning 


def sink_listener(self,sink_t, timeout):
    '''
    if a path to sink is contructed a drone of the niegboor wil become irremovable 
    when a drone became irremovable it will send message contains its id to all around 

    The sink listener check that message contains S- id different than id of sink 
    and based on that it will be considered as a message to complet the path  
     
    '''
    while not sink_t.end_of_spanning.is_set(): # the end is not reached , keep listenning
        msg= self.retrive_msg_from_buffer() 
        
        # Receiving message asking for data 
        if msg == Demand_header.encode() + b'\n':
            data_msg= self.build_spot_info_message(Reponse_header) # Build message that contains all data
            self.send_msg(data_msg)

        # Receiving message containing data     
        if msg.startswith(Reponse_header.encode()) and msg.endswith(b'\n'):
            self.neighbors_list_updated = threading.Event()
            positionX, positionY, state, id_value= self.decode_spot_info_message(msg)
            self.update_neighbors_list(positionX, positionY, state, id_value )
            self.neighbors_list_updated.set()
            
        id_rec=0 
        if id_rec != self.id: 
            print("Message received!")
            with sink_t.lock_sink:  # Acquire the lock
                sink_t.message_counter += 1
                sink_t.remaining_time = sink_t.timeout
    
    sink_t.end_of_spanning.clear() # reset so the next spanning the listenning loop will be activated  


def spanning_sink(self):
    # initialize a timer that will be rest after reciving new message 
    sink_proess = Sink_Timer()
    sink_proess.run()
 


# Lock for the shared dictionary
lock = threading.Lock()

# Event flag to signal that xbee_listener wants to write
listener_neighbor_update_state = threading.Event()
listener_current_updated_irremovable = threading.Event()
listener_end_of_spanning = threading.Event()

def xbee_listener(self):
    '''
    1- check that the message starts with S ( for spanning ) otherwise ignore and relance spaning 
    2- check in the message what is the data ( it should be id )
    3- extract the id and save it in id_msg and the state of the sender that it should be usually irremovabe 
        The reason is that the messages in this pahse are sent only if the drone is irremovable 
    '''
    # Keep litining until reciving a end of the phase 
    while not listener_end_of_spanning.is_set(): 
        msg= self.retrive_msg_from_buffer() 
        
        # Receiving message asking for data 
        if msg == Demand_header.encode() + b'\n':
            data_msg= self.build_spot_info_message(Reponse_header) # Build message that contains all data
            self.send_msg(data_msg)

        # Receiving message containing data     
        if msg.startswith(Reponse_header.encode()) and msg.endswith(b'\n'):
            self.neighbors_list_updated = threading.Event()
            positionX, positionY, state, id_value= self.decode_spot_info_message(msg)
            self.update_neighbors_list(positionX, positionY, state, id_value )
            self.neighbors_list_updated.set()


        if id_msg == self.id : 
            #that means one of the neigboor is irremovable and sent a taged message
            # This means that this current drone should change it is current state 
            with lock:
                if self.state== Border: 
                    self.change_state(Irremovable_boarder)
                    self.neighbor_list[id_msg]['state']=Irremovable_boarder # here you update the value
                else: 
                    self.change_state(Irremovable)
                    self.neighbor_list[id_msg]['state']=Irremovable# here you update the value
                listener_current_updated_irremovable.set() # flag that the state was changed to irremovable 

        else: # the recieved msg refer to changes in state to irrremovable in one of the nighbors
            # Signal that we want to write
            listener_neighbor_update_state.set()
            with lock:
                self.neighbor_list[id_msg]['state']=1 # here you update the value
            listener_neighbor_update_state.clear()  # Clear the flag

        if id_msg ==-127: # it is the message sent by the sink announcing the end of spanning phase
            listener_end_of_spanning.set()


# since the 6 neighbors has 2 different regions one close to sink and one in opposit 
# check the 3 neighbors that are close to the sink to know if any is irremovable
def find_close_neigboor_2sink(self):
    neighbor_irremovable= None
    filtered_neighbors= None

    # the search should be only in the neighbor_list but since it contains also the data of s0 ( current drone)
    # Because we need to find min of occupied distance, we should remove the s0 from the list 
    neighbor_list_no_s0= self.neighbor_list[1:]

    # Sort the neighbor_list based on the distance
    sorted_neighbors = sorted(neighbor_list_no_s0, key=lambda x: x['distance'])

    # Extract the first 3 elements with the minimum distances
    three_min_neighbors = sorted_neighbors[:3]

    # Filter the neighbor_list for objects with "drones_in" > 0 as it contains a drone in 
    # it is done after sorting because we need the occupied of the 3 minimum , ( if it is done before it would be minimum distance of occupied spots)
    filtered_neighbors = [neighbor for neighbor in three_min_neighbors if neighbor["drones_in"] > 0]
    
    if filtered_neighbors is not None:  # there are occupied neighbors
        # Find the neighbor with id of the drone that is irremovable or it is sink ( distance =0 )
        # if you arrive to irremovable or there is no need to send message
        
        # here a mutex needed in this operation to read  the state is already changed 
        
        # lock prioirty fo the state
        # Check if xbee_listener wants to write
        if not listener_neighbor_update_state.is_set():
            # Acquire the lock to read the shared dictionary
            with lock:
                neighbor_irremovable = next((neighbor for neighbor in filtered_neighbors if ( neighbor['state'] == Irremovable) ), None) # no need to check for or neighbor['distance'] == 0 because the sink is already irremovable 

        if neighbor_irremovable== None:
            return min(filtered_neighbors, key=lambda x: x["distance"])["id"] # retuen the id of the drone 
        else: 
            return -1
    else: # the case of no occupied neighbors around is very possible after further expansion
        # check if the neighbors drones are preivious border 
        # befor this function is called all niegbor data are updates    
            # at the same time the neighbors drones contains in neighbor_list info of its own neighbor
            # so the each drone in this pahse need to check if it has a drone was a border
        # notice the earch here for all niegbors 
        # "border -irremovable" because it can turn to only irrmovable if it is not part of the border    
        
        # checking for previous need lock also because it is related to the chnages of sate 
        if not listener_neighbor_update_state.is_set():
            # Acquire the lock to read the shared dictionary
            with lock:
                drone_previous_border = [neighbor for neighbor in self.neighbor_list if ( neighbor["drones_in"] > 0 and neighbor["previous_state"]== Border or neighbor["previous_state"]== Irremovable_boarder ) ] 
        
        # if there are many drones had a state border befor chose the one closer to sink 
        if drone_previous_border>1: 
            return min(drone_previous_border, key=lambda x: x["distance"])["id"]
        elif drone_previous_border==1:
            return drone_previous_border["id"]

        #nohthing found then connect to the border it is not possible 
        # after the further exapnsion, the drones will be close to the old border 
        # it is necessary to find a free drone or previous border 
        # TODO check of it 


def find_close_neigboor_2border(self):
    neighbor_irremovable= None
     
    neighbor_list_no_s0= self.neighbor_list[1:]

    # Sort the neighbor_list based on the distance
    sorted_neighbors = sorted(neighbor_list_no_s0 , key=lambda x: x['distance'],reverse=True)

    # Extract the first 3 elements with the max distances ( close to the border)
    three_max_neighbors = sorted_neighbors[:3]

    # Filter the neighbor_list for objects with "drones_in" > 0 as it contains a drone in 
    filtered_neighbors = [neighbor for neighbor in three_max_neighbors if neighbor["drones_in"] > 0]
    
    # Find the neighbor with drone that is irremovable or irremovable- border because 
    # if you arrive to irremovable or irremovable-border there is no need to send message
    # here a mutex needed in this operation to read  the state is already changed 
    
    # Check if xbee_listener wants to write( messag is recived)
    if not listener_neighbor_update_state.is_set():
        # Acquire the lock to read the shared dictionary
        with lock:     
            neighbor_irremovable = next((neighbor for neighbor in filtered_neighbors if ( neighbor['state'] == Irremovable or neighbor['state'] == Irremovable_boarder) ), None)

    if neighbor_irremovable== None: # if it doesnt exist check what is closest to the sink 
        return max(filtered_neighbors, key=lambda x: x["distance"])["id"] # retuen the id of the drone 
    else: # there is a irremovable drone in occupied neighbors
        return -1
  

def build_path(self):

    #if (self.state== Irremovable) or (self.state == Irremovable_boarder):
        send_msg_drone_id= self.find_close_neigboor_2sink() 
        if send_msg_drone_id != -1 : # there is no irremovable send msg to a drone close to sink to make it irremovable 
            self.drone_id_to_sink=send_msg_drone_id # save the id of the drone for future use to connect to sink
            # send message to a drone that had Id= send_msg_drone_id
        
        if self.state != Irremovable_boarder:  # it is irrmovable doesnt belong to boarder no need to check (self.state != " irremovable- border" ) 
            send_msg_drone_id= self.find_close_neigboor_2border()  # since it doesnt belong to border then find to path to border 
            if send_msg_drone_id != -1 : # there is no irremovable send msg to a drone to make it irremovable 
                self.drone_id_to_border= send_msg_drone_id # save the drone id that is the path to the border from the current one 
                # send message to a drone that had Id= send_msg_drone_id
    

def spanining ( self, vehicle ): 
    #Stay hovering while spanning is done 
    hover(vehicle)

    # update neigboors after the finish of expansion 
    self.build_data_demand_message()
    self.neighbors_list_updated.wait()
    self.neighbors_list_updated.clear()


    # since a lock is introduced then the drone will not read a old value after any update it will see it 
    # this shuld be different from only normal lisenting because it cotains listener
    xbee_thread = threading.Thread(target=xbee_listener, args=(self,)) #pass the function reference and arguments separately to the Thread constructor.
    xbee_thread.start()

    # for the sink drone TODO the sink is responsable of ending the phase  
    if self.spot['distance']==0: # if the drone is sink ( leader of the termination of the spaning phase)
        self.spanning_sink()
    
    else: 
        if not listener_neighbor_update_state.is_set():
            # Acquire the lock to read the shared dictionary            
            # the drone is free 
            # wait for turning to a irremovable by another drone that build a path or wait broadcast from sink of finihing spannng
            if self.state== Free or self.state== Border:
                # Wait for xbee_listener to signal that state has been changed ( doesn't keep the CPU busy.)
                while not listener_current_updated_irremovable.is_set() or ( not listener_end_of_spanning.is_set()):
                    time.sleep(0.1)
                listener_current_updated_irremovable.clear() # need to clear it for the next spanning 
        if not listener_neighbor_update_state.is_set():
            # for the rest of the drones 
            if (self.state== Irremovable) or (self.state == Irremovable_boarder): 
                self.build_path()

        listener_end_of_spanning.wait() # wait until reciving end to finish this phase , Note if the end detected in Free then this will not wait because it is already seen 
        listener_end_of_spanning.clear()

        # send the end of the phase from your side to every one around 

    xbee_thread.join() # this top at the end of he phase because no need to use mutex anymore 
    set_to_move(vehicle) # Drone can move now in the balancing phase 

