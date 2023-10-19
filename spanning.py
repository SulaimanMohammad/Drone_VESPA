from  expan import *
import struct
import sys
import threading
import time

'''
-------------------------------------------------------------------------------------
---------------------------------- Communication ------------------------------------
-------------------------------------------------------------------------------------
'''
Border_sink_confirm= -1
spanning_terminator=-127 

def build_target_message(self,target_id, data=0):
    max_byte_count = self.determine_max_byte_size(target_id)
    message = Spanning_header.encode() + struct.pack('>B', max_byte_count)    
    message += target_id.to_bytes(max_byte_count, 'big')
    
    data_byte_size = self.determine_max_byte_size(data)
    data_bytes = data.to_bytes(data_byte_size, 'big')
    message += struct.pack('>B', data_byte_size) 
    message += data_bytes  
    
    message += b'\n'
    return message

def decode_target_message(message):
    # Extract max_byte_count and target_id
    max_byte_count = struct.unpack('>B', message[2:3])[0]
    target_id_start = 3
    target_id_end = target_id_start + max_byte_count
    target_id = int.from_bytes(message[target_id_start:target_id_end], 'big')
    
    # Extract the data size and data
    data_size_start = target_id_end
    data_size_end = data_size_start + 1  # Since data size is stored in 1 byte
    data_size = struct.unpack('>B', message[data_size_start:data_size_end])[0]
    
    # Extract and convert the data bytes to integer
    data_start = data_size_end
    data_end = data_start + data_size
    data_bytes = message[data_start:data_end]
    data = int.from_bytes(data_bytes, 'big')
    
    return target_id, data

def forward_confirm_msg(self):
    for id in self.drone_id_to_sink:
        msg_border_sink= self.build_target_message(id,Border_sink_confirm)
        self.send_msg(msg_border_sink)

def append_id_to_path(list_to_append, ids):
    # If ids is a single integer, convert it to a list
    if isinstance(ids, int):
        ids = [ids]
    # Append elements from ids to list_to_append without repetition
    for id in ids:
        if id not in list_to_append:
            list_to_append.append(id)

'''
-------------------------------------------------------------------------------------
---------------------------------- Sink Procedure------------------------------------
-------------------------------------------------------------------------------------
'''
class Sink_Timer:
    def __init__(sink_t,self,timeout=10):
        sink_t.timeout = timeout
        sink_t.message_counter = 0  # Initialize message count
        sink_t.remaining_time = sink_t.timeout
        sink_t.lock_sink = threading.Lock()  # Create a lock
        sink_t.message_thread = threading.Thread(target=sink_listener, args=(sink_t,self,timeout,))
        sink_t.message_thread.start()
        sink_t.end_of_spanning= threading.Event()

        # Needed to find what neigbour that became irremvable due to finding target
        self.demand_neighbors_info() 
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
        # Called when the timer reaches its timeout without being reset.
        print("Time's up! ")
        self.change_state(Irremovable) # The sink will always be irremovable 
        # No message received, thus the sink must build path to the border 
        if sink_t.message_counter==0: 
            target_id= self.find_close_neigboor_2border() 
            if target_id != -1 : # No irremovable send msg to a drone to make it irremovable 
                # Send message to a drone that had Id= target_id
                append_id_to_path( self.drone_id_to_border, target_id ) 
                msg= self.build_target_message(target_id)
                self.send_msg(msg)

        # If drone already have message and path is constructed then it needs to wait the message flow from border to sink 
        # and if the sink needed to constrcut the path starting from itself , still need wait a message from border to come back   
        sink_t.end_of_spanning.wait() 
        sink_t.end_of_spanning.clear() 
        sink_listener.join() 

def sink_listener(self,sink_t, timeout):
    '''
    if a path to sink is constructed a drone of the niegboor wil become irremovable 
    when a drone became irremovable it will send message contains its id to all around
    And try to find how to arrive to the sink if it is found then message will be sent to it conatins sink id. 

    The sink listener check that message id= id of sink 
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
            self.get_neighbors_info()

        if msg.startswith(Spanning_header.encode()) and msg.endswith(b'\n'):
            id_rec,data = self.decode_target_message(msg)
            if id_rec == self.id: 
                if data ==0:
                    with sink_t.lock_sink:  # Acquire the lock
                        sink_t.message_counter += 1
                        sink_t.remaining_time = sink_t.timeout
                # Data refere that the message came from border so path constructed 
                elif data== Border_sink_confirm:
                    # Sink sends the end of spanning
                    end_msg= self.build_target_message(spanning_terminator)
                    self.send_msg(end_msg)
                    sink_t.end_of_spanning.set()
            else: # Recieved msg refer to changes in state to irrremovable in one of the nighbors
                self.update_state_in_neighbors_list(id_rec, Irremovable)
                # Save the ids of drone that is (irremovable) has path to border 
                # All drones arounf the sink are to the border 
                append_id_to_path( self.drone_id_to_border, id_rec)   

def spanning_sink(self):
    # initialize a timer that will be rest after reciving new message 
    sink_proess = Sink_Timer()
    sink_proess.run()

'''
-------------------------------------------------------------------------------------
------------------------------------ Build path -------------------------------------
-------------------------------------------------------------------------------------
'''
# Event flag to signal that xbee_listener wants to write
listener_current_updated_irremovable = threading.Event()
listener_end_of_spanning = threading.Event()
def xbee_listener(self):
    '''
    1-If the header refer to data demand (build the message that contains the data and send it)
    2- Header refers to data receive it, then decode the message and update the information of the neighbour 
    3- Header is Spanning (Header to build the path to sink and border)
        * decode the message 
        * if the message conatins the id of the current drone then it is targeted to be part of the path (Irremovable) by one of neighbour.
            - change the state and rise flag that the drone changed its state
        * if the message contains another id, this referes that the owner of this id changed its state to (Irremovable) and must update the local info about this drone.
          that is important so the current drone check if any neighbour is part of the path so the process will stop.
        * if the id= -127 that refers to the brodcast of ending the spanning phase 
    '''
    # Keep listening until reciving a end of the phase 
    while not listener_end_of_spanning.is_set(): 
        msg= self.retrive_msg_from_buffer() 
        
        # Receiving message asking for data 
        if msg == Demand_header.encode() + b'\n':
            data_msg= self.build_spot_info_message(Reponse_header) # Build message that contains all data
            self.send_msg(data_msg)

        # Receiving message containing data     
        if msg.startswith(Reponse_header.encode()) and msg.endswith(b'\n'):
            self.get_neighbors_info()
        
        # Message of building the path 
        if msg.startswith(Spanning_header.encode()) and msg.endswith(b'\n'):
            id_rec, data= self.decode_target_message(msg)           
            if id_rec == self.id :
                if data==0:
                    if self.state== Border: 
                        self.change_state(Irremovable_boarder)
                    else: 
                        self.change_state(Irremovable)
                    # Send message to the neighbors to inform the new changes 
                    msg= self.build_target_message(self.id)
                    self.send_msg(msg)
                    listener_current_updated_irremovable.set() # Flag that the state was changed to irremovable 
                elif data == Border_sink_confirm: 
                    # It's a meaage flow from border to sink to verfiry the path and end the pahse
                    # verfiy if became already irremovable drone (part of the path)
                    if (self.state == Irremovable):
                        self.forward_confirm_msg()

            elif id_rec == spanning_terminator: # Message sent by the sink announcing the end of spanning phase
                end_msg= self.build_target_message(spanning_terminator)
                self.send_msg(end_msg)
                listener_end_of_spanning.set()
            
            else: # Recieved msg refer to changes in state to irrremovable in one of the nighbors
                self.update_state_in_neighbors_list(id_rec, Irremovable) 

# since the 6 neighbors has 2 different regions one close to sink and one in opposit 
# check the 3 neighbors that are close to the sink to know if any is irremovable
def find_close_neigboor_2sink(self):
    neighbor_irremovable= None
    filtered_neighbors= None

    # The search should be only in the neighbor_list but since it contains also the data of s0 (current spot)
    # But because we need to find min of occupied distance, we should remove the s0 from the list 
    with self.lock_neighbor_list:
        neighbor_list_no_s0= self.neighbor_list[1:]

    # Sort the neighbor_list based on the distance
    reference_distance = self.neighbor_list[0]['distance']

    # Filter neighbors with distance from sink less than current spot and sort them
    sorted_neighbors = sorted(
    [neighbor for neighbor in neighbor_list_no_s0 if neighbor['distance'] < reference_distance],
    key=lambda x: x['distance'])
    
    # Extract the first 3 elements with the minimum distances
    three_min_neighbors = sorted_neighbors[:3]

    # Filter the neighbor_list with minimum distance the spot should be occupied "drones_in" > 0  
    filtered_neighbors = [neighbor for neighbor in three_min_neighbors if neighbor["drones_in"] > 0]
    
    if filtered_neighbors is not None:  # There are occupied neighbors
        # Find the neighbor with id of the drone that is irremovable 
        # if you arrive to irremovable 

        with self.lock_neighbor_list:
            neighbor_irremovable = next((neighbor for neighbor in filtered_neighbors if ( neighbor['state'] == Irremovable) ), None)

        # No irremovable found 
        if neighbor_irremovable== None:
            return min(filtered_neighbors, key=lambda x: x["distance"])["id"] # retuen the id of the drone 
        else: 
            # irremovable found, save it
            append_id_to_path( self.drone_id_to_sink, neighbor_irremovable )
            return -1
    else: 
        # The case of no occupied neighbors close to sink around is very possible after further expansion
        # Check if any of the neighbors drones are preivious border 
        with self.lock_neighbor_list:
            drone_previous_border = [neighbor for neighbor in self.neighbor_list if ( neighbor["drones_in"] > 0 and neighbor["previous_state"]== Border or neighbor["previous_state"]== Irremovable_boarder ) ] 
        
        # If there are many drones had a state border befor chose the one closer to sink 
        if drone_previous_border>1: 
            return min(drone_previous_border, key=lambda x: x["distance"])["id"]
        elif drone_previous_border==1:
            return drone_previous_border["id"]

        # Founding no drone it is not possible, after the further exapnsion, the drones will be between the old border an dthe new one 
  
def find_close_neigboor_2border(self):
    neighbor_irremovable= None
    
    with self.lock_neighbor_list:  
        neighbor_list_no_s0= self.neighbor_list[1:]

    reference_distance = self.neighbor_list[0]['distance']

    # Filter neighbors with distance from sink bigger than current spot and sort them
    sorted_neighbors = sorted(
    [neighbor for neighbor in neighbor_list_no_s0 if neighbor['distance'] > reference_distance],
    key=lambda x: x['distance'])
    
    # Extract the first 3 elements with the max distances (close to the border)
    three_max_neighbors = sorted_neighbors[:3]

    # Filter the neighbor_list for objects with "drones_in" > 0 as it contains a drone in 
    filtered_neighbors = [neighbor for neighbor in three_max_neighbors if neighbor["drones_in"] > 0]
    
    # Find the neighbor with drone that is irremovable or irremovable- border because 
    # if there is a irremovable or irremovable-border there is no need to send message
    # Here a mutex needed in this operation to read  the state
    with self.lock_neighbor_list:     
        neighbor_irremovable = next((neighbor for neighbor in filtered_neighbors if ( neighbor['state'] == Irremovable or neighbor['state'] == Irremovable_boarder) ), None)

    if neighbor_irremovable== None: # No irremovable found, check what is the closest to the sink
        return max(filtered_neighbors, key=lambda x: x["distance"])["id"] # Retuen the id of the drone 
    else: # There is a irremovable drone in occupied neighbors but usually it is only one
        append_id_to_path(self.drone_id_to_border, neighbor_irremovable)
        return -1 
  
def build_path(self):
    # Irremovable_boarder and Irremovable needs to build bath to the sink 
    target_id= self.find_close_neigboor_2sink() 
    if target_id != -1 : # No irremovable around send msg to a drone close to sink to make it irremovable 
        # save the id of the drone for future use to connect to sink
        append_id_to_path(self.drone_id_to_sink, target_id)
        msg= self.build_target_message(target_id)
        self.send_msg(msg)
    
    if self.state != Irremovable_boarder:  # it is irrmovable doesnt belong to boarder no need to check (self.state != " irremovable- border" ) 
        target_id= self.find_close_neigboor_2border()  # since it doesnt belong to border then find to path to border 
        if target_id != -1 : 
            append_id_to_path(self.drone_id_to_border, target_id)
            msg= self.build_target_message(target_id)
            self.send_msg(msg)
        
def spanining ( self, vehicle ): 
    # Stay hovering while spanning communication 
    hover(vehicle)

    if self.spot['distance']==0: # if the drone is sink ( leader of the termination of the spaning phase)
        self.spanning_sink()
    
    else:
        xbee_thread = threading.Thread(target=xbee_listener, args=(self,))
        xbee_thread.start()

        # Update neigboors info after the end of expansion and wait the data to be recived
        # Needed to find what neigbour that became irremvable due to finding target 
        self.demand_neighbors_info()

        # Free drone wait for msg to become irremovable by another drone or wait broadcast from sink of finihing Spainning
        if self.state== Free or self.state== Border:
            # Wait for xbee_listener to signal that state has been changed ( doesn't keep the CPU busy.)
            while not listener_current_updated_irremovable.is_set() or ( not listener_end_of_spanning.is_set()):
                time.sleep(1)
            listener_current_updated_irremovable.clear() # need to be cleared for the next spanning 
        
        # For Irremovables and Free drones that were changed  
        if (self.state== Irremovable) or (self.state == Irremovable_boarder): 
            self.build_path()
        
        # Send a message that will travel from border to sink and that will annouce end of the pahse 
        if self.state== Irremovable_boarder:
            while not self.drone_id_to_sink: # wait until list not empty 
                time.sleep(1)
            self.forward_confirm_msg()
        
        # Wait until reciving end to finish this phase 
        listener_end_of_spanning.wait() 
        listener_end_of_spanning.clear()
        
        # Stop listener
        xbee_thread.join() 
        self.clear_buffer()
    
    if self.state == Free:
        set_to_move(vehicle) # only free Drone can move now in the balancing phase 
