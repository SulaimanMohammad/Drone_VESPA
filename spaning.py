from  expan import *
import struct
import sys
import threading
import time


def build_target_message(self, target_id):
    max_byte_count = self.determine_max_byte_size(target_id)
    message = Spanning_header.encode() + struct.pack('>B', max_byte_count)    
    message += target_id.to_bytes(max_byte_count, 'big')
    message += b'\n'
    return message

def decode_target_message(self,message):    
    max_byte_count = struct.unpack('>B', message[1:2])[0]
    target_id_start = 2
    target_id_end = target_id_start + max_byte_count
    target_id = int.from_bytes(message[target_id_start:target_id_end], 'big')
    return target_id


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

        if msg.startswith(Spanning_header.encode()) and msg.endswith(b'\n'):
            id_rec= self.decode_target_message(msg)
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
 



# Event flag to signal that xbee_listener wants to write
listener_current_updated_irremovable = threading.Event()
listener_end_of_spanning = threading.Event()

def xbee_listener(self):
    '''
    1-If the header refer to data demand (build the message that contains the data and send it)
    2- Header refers to data receive it, then decode the message and update the information of the neighbour 
    3- Header is Spanning (Header to build the path to sink and border)
        * decode the message 
        * if the message conatins the id of the current drone thnen it is targeted to be part of the path (Irremovable) by one of neighbour.
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
            self.neighbors_list_updated = threading.Event()
            time.sleep(1) # Wait to have all msgs 
            # Retrive all the reponse messages then rise the flage that all is received 
            while msg.startswith(Reponse_header.encode()):
                positionX, positionY, state, id_value= self.decode_spot_info_message(msg)
                self.update_neighbors_list(positionX, positionY, state, id_value )
                msg= self.retrive_msg_from_buffer()

            self.neighbors_list_updated.set()
        
        # Message of building the path 
        if msg.startswith(Spanning_header.encode()) and msg.endswith(b'\n'):
            id_rec= self.decode_target_message(msg)
            if id_rec == self.id : 
                if self.state== Border: 
                    self.change_state(Irremovable_boarder)
                else: 
                    self.change_state(Irremovable)
                listener_current_updated_irremovable.set() # Flag that the state was changed to irremovable 
            
            else: # Recieved msg refer to changes in state to irrremovable in one of the nighbors
                self.update_state_in_neighbors_list(id_rec, Irremovable) 

            if id_rec ==-127: # Message sent by the sink announcing the end of spanning phase
                listener_end_of_spanning.set()


# since the 6 neighbors has 2 different regions one close to sink and one in opposit 
# check the 3 neighbors that are close to the sink to know if any is irremovable
def find_close_neigboor_2sink(self):
    neighbor_irremovable= None
    filtered_neighbors= None

    # The search should be only in the neighbor_list but since it contains also the data of s0 (current spot)
    # But because we need to find min of occupied distance, we should remove the s0 from the list 
    with self.lock:
        neighbor_list_no_s0= self.neighbor_list[1:]

    # Sort the neighbor_list based on the distance
    sorted_neighbors = sorted(neighbor_list_no_s0, key=lambda x: x['distance'])

    # Extract the first 3 elements with the minimum distances
    three_min_neighbors = sorted_neighbors[:3]

    # Filter the neighbor_list with minimum distance the spot should be occupied "drones_in" > 0  
    filtered_neighbors = [neighbor for neighbor in three_min_neighbors if neighbor["drones_in"] > 0]
    
    if filtered_neighbors is not None:  # There are occupied neighbors
        # Find the neighbor with id of the drone that is irremovable or it is sink ( distance =0 )
        # if you arrive to irremovable 

        with self.lock:
            neighbor_irremovable = next((neighbor for neighbor in filtered_neighbors if ( neighbor['state'] == Irremovable) ), None) # no need to check for or neighbor['distance'] == 0 because the sink is already irremovable 

        if neighbor_irremovable== None:
            return min(filtered_neighbors, key=lambda x: x["distance"])["id"] # retuen the id of the drone 
        else: 
            return -1
    else: 
        # The case of no occupied neighbors close to sink around is very possible after further expansion
        # Check if any of the neighbors drones are preivious border 
        with self.lock:
            drone_previous_border = [neighbor for neighbor in self.neighbor_list if ( neighbor["drones_in"] > 0 and neighbor["previous_state"]== Border or neighbor["previous_state"]== Irremovable_boarder ) ] 
        
        # If there are many drones had a state border befor chose the one closer to sink 
        if drone_previous_border>1: 
            return min(drone_previous_border, key=lambda x: x["distance"])["id"]
        elif drone_previous_border==1:
            return drone_previous_border["id"]

        # Founding no drone it is not possible, after the further exapnsion, the drones will be close to the old border 
  
def find_close_neigboor_2border(self):
    neighbor_irremovable= None
    
    with self.lock:  
        neighbor_list_no_s0= self.neighbor_list[1:]

    # Sort the neighbor_list based on the distance
    sorted_neighbors = sorted(neighbor_list_no_s0 , key=lambda x: x['distance'],reverse=True)

    # Extract the first 3 elements with the max distances (close to the border)
    three_max_neighbors = sorted_neighbors[:3]

    # Filter the neighbor_list for objects with "drones_in" > 0 as it contains a drone in 
    filtered_neighbors = [neighbor for neighbor in three_max_neighbors if neighbor["drones_in"] > 0]
    
    # Find the neighbor with drone that is irremovable or irremovable- border because 
    # if there is a irremovable or irremovable-border there is no need to send message
    # Here a mutex needed in this operation to read  the state
    with self.lock:     
        neighbor_irremovable = next((neighbor for neighbor in filtered_neighbors if ( neighbor['state'] == Irremovable or neighbor['state'] == Irremovable_boarder) ), None)

    if neighbor_irremovable== None: # No irremovable found, check what is the closest to the sink 
        return max(filtered_neighbors, key=lambda x: x["distance"])["id"] # retuen the id of the drone 
    else: # There is a irremovable drone in occupied neighbors
        return -1
  
def build_path(self):
    # Irremovable_boarder and Irremovable needs to build bath to the sink 
    send_msg_drone_id= self.find_close_neigboor_2sink() 
    if send_msg_drone_id != -1 : # there is no irremovable send msg to a drone close to sink to make it irremovable 
        self.drone_id_to_sink=send_msg_drone_id # save the id of the drone for future use to connect to sink
        # send message to a drone that had Id= send_msg_drone_id
        msg= self.build_target_message(send_msg_drone_id)
        self.send_msg(msg)
    
    if self.state != Irremovable_boarder:  # it is irrmovable doesnt belong to boarder no need to check (self.state != " irremovable- border" ) 
        send_msg_drone_id= self.find_close_neigboor_2border()  # since it doesnt belong to border then find to path to border 
        if send_msg_drone_id != -1 : # there is no irremovable send msg to a drone to make it irremovable 
            self.drone_id_to_border= send_msg_drone_id # save the drone id that is the path to the border from the current one 
            # send message to a drone that had Id= send_msg_drone_id
            msg= self.build_target_message(send_msg_drone_id)
            self.send_msg(msg)
        
def spanining ( self, vehicle ): 
    #Stay hovering while spanning communication 
    hover(vehicle)

    xbee_thread = threading.Thread(target=xbee_listener, args=(self,))
    xbee_thread.start()

    # Update neigboors info after the end of expansion and wait the data to be recived
    # Needed to find what neigbour that became irremvable due to finding target 
    self.build_data_demand_message()
    self.neighbors_list_updated.wait()
    self.neighbors_list_updated.clear()

    # for the sink drone TODO the sink is responsable of ending the phase  
    if self.spot['distance']==0: # if the drone is sink ( leader of the termination of the spaning phase)
        self.spanning_sink()
    
    else: 

        # Free drone wait for msg to become irremovable by another drone or wait broadcast from sink of finihing Spainning
        if self.state== Free or self.state== Border:
            # Wait for xbee_listener to signal that state has been changed ( doesn't keep the CPU busy.)
            while not listener_current_updated_irremovable.is_set() or ( not listener_end_of_spanning.is_set()):
                time.sleep(1)
            listener_current_updated_irremovable.clear() # need to be cleared for the next spanning 
        
        # For Irremovables and Free drones that were changed  
        if (self.state== Irremovable) or (self.state == Irremovable_boarder): 
            self.build_path()

        listener_end_of_spanning.wait() # wait until reciving end to finish this phase , Note if the end detected in Free then this will not wait because it is already seen 
        listener_end_of_spanning.clear()


    xbee_thread.join() # this top at the end of he phase because no need to use mutex anymore 
    
    if self.state == Free:
        set_to_move(vehicle) # only free Drone can move now in the balancing phase 

