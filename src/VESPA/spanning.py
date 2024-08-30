from .VESPA_module import *

set_env(globals())
'''
-------------------------------------------------------------------------------------
---------------------------------- Communication sink------------------------------------
-------------------------------------------------------------------------------------
'''
Border_sink_confirm= -1
spanning_terminator=-127


def build_target_message(target_id, header=Spanning_header, data=0):
    max_byte_count = determine_max_byte_size (target_id)
    message = header.encode() + struct.pack('>B', max_byte_count)
    message += target_id.to_bytes(max_byte_count,  byteorder='big',signed=True)
    data_byte_size = determine_max_byte_size(data)
    # Convert to bytes using two's complement
    data_bytes = data.to_bytes(data_byte_size, byteorder='big', signed=True)
    message += struct.pack('>B', data_byte_size)
    message += data_bytes
    message += b'\n'
    return message

def decode_target_message(message):
    # Extract max_byte_count and target_id
    content = message[1:]
    max_byte_count = struct.unpack('>B', content[:1])[0]
    target_id_start = 2
    target_id_end = target_id_start + max_byte_count
    target_id = int.from_bytes(message[target_id_start:target_id_end], 'big',signed=True)
    # Extract the data size and data
    data_size_start = target_id_end
    data_size_end = data_size_start + 1  # Since data size is stored in 1 byte
    data_size = int.from_bytes(message[data_size_start:data_size_end], 'big')
    # Extract and convert the data bytes to integer
    data_start = data_size_end
    data_end = data_start + data_size
    data_bytes = message[data_start:data_end]
    # Decode the data bytes as a signed integer
    data = int.from_bytes(data_bytes, 'big', signed=True)
    return target_id, data


# This func uses build_target_message but with data=!0 to be recognized as message to confirm the path 
def forward_confirm_msg(self, data, header=Spanning_header):
    # If data is Border_sink_confirm then it is for confirmation with spanning header but there is another way is to send header to terminate the algorithm 
    # In this way data will be 0 and the hedear will be different 
    for id in self.drone_id_to_sink:
        msg_border_sink= build_target_message(id,header,data)
        send_msg(msg_border_sink)

def append_id_to_path(list_to_append, ids):
    # If ids is a single integer, convert it to a list
    if isinstance(ids, int):
        ids = [ids]
    # Append elements from ids to list_to_append without repetition
    for id in ids:
        if id not in list_to_append:
            list_to_append.append(id)

def path_around_exist(self):
    filtered_neighbors = [neighbor for neighbor in self.get_neighbor_list()[1:] if neighbor["drones_in"] > 0]
    if filtered_neighbors is not None:  # There are occupied neighbors
            neighbor_irremovable = next((neighbor for neighbor in filtered_neighbors if ( neighbor['states'] == Irremovable) ), None)
    if (neighbor_irremovable is not None) and neighbor_irremovable>0:
        return True
    else: 
        return False
    

def check_continuity_of_listening(self):
    if listener_end_of_spanning.is_set():
        return False 
    else: 
        return True
    
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
        sink_t.message_thread = threading.Thread(target=sink_listener, args=(sink_t,self,))
        sink_t.message_thread.start()

        # Needed to find what neigbour that became irremvable due to finding target
        self.demand_neighbors_info() 
    def run(sink_t, self):
        while True:
            with sink_t.lock_sink:  # Acquire the lock
                sink_t.remaining_time -= 0.5
                if sink_t.remaining_time <= 0:
                    sink_t.time_up(self)
                    break
            time.sleep(0.5)

    def time_up(sink_t,self):
        # Called when the timer reaches its timeout without being reset.
        write_log_message("Time's up for the sink counter! ")
        # The sink will always be irremovable 
        if self.get_state() == Border:
            self.change_state_to(Irremovable_boarder) 
        else:
            self.change_state_to(Irremovable) 
        
        if self.get_state()== Irremovable_boarder:
            # Sink is part of the border also then send end of spanning 
            end_msg= build_target_message(spanning_terminator)
            send_msg(end_msg)
            listener_end_of_spanning.set()
        else:
            # No message received, thus the sink must build path to the border 
            if sink_t.message_counter==0 or (not path_around_exist(self)): 
                write_log_message("No path message has been received and no path found")
                target_id= find_close_neigboor_2border(self)
                write_log_message(f"Build path from sink to border through drone {target_id}")
                if target_id != -1 : # No irremovable send msg to a drone to make it irremovable 
                    # Send message to a drone that had Id= target_id
                    append_id_to_path( self.drone_id_to_border, target_id ) 
                    msg= build_target_message(target_id)
                    send_msg(msg)

        sink_t.message_thread.join() 
        # Sink should also go to the balancing phase in case another will move over it and be part of the circular communication in case it is a border


def sink_listener(sink_t, self):
    '''
    if a path to sink is constructed a drone of the niegboor wil become irremovable 
    when a drone became irremovable it will send message contains its id to all around
    And try to find how to arrive to the sink if it is found then message will be sent to it conatins sink id. 

    The sink listener check that message id= id of sink 
    and based on that it will be considered as a message to complet the path  
     
    '''
    while check_continuity_of_listening(self): # the end is not reached , keep listenning
        try: 
            msg= retrieve_msg_from_buffer(listener_end_of_spanning) 
            if msg.startswith(Emergecy_header.encode()) and msg.endswith(b'\n'):
                self.emergency_stop()
                break
            self.exchange_neighbors_info_communication(msg)
            if msg.startswith(Spanning_header.encode()) and msg.endswith(b'\n'):
                id_rec,data = decode_target_message(msg)
                if id_rec == self.id: 
                    if data ==0:
                        with sink_t.lock_sink:  # Acquire the lock
                            sink_t.message_counter += 1
                            sink_t.remaining_time = sink_t.timeout
                    # Data refere that the message came from border so path constructed 
                    elif data== Border_sink_confirm:
                        # Sink sends the end of spanning
                        end_msg= build_target_message(spanning_terminator)
                        send_msg(end_msg)
                else: # Recieved msg refer to changes in state to irrremovable in one of the nighbors
                    self.update_state_in_neighbors_list(id_rec, Irremovable)
                    # Save the ids of drone that is (irremovable) has path to border 
                    # All drones arounf the sink are to the border 
                    append_id_to_path( self.drone_id_to_border, id_rec)

            elif  msg.startswith(Info_header.encode()) and msg.endswith(b'\n'):
                self.forward_data_message(msg, self.drone_id_to_sink[0])

            else: 
                continue
        except:
            write_log_message("Thread sink_listener spanning Interrupt received, stopping...")
            self.emergency_stop()   
                

def spanning_sink(self):
    # initialize a timer that will be rest after reciving new message 
    sink_proess = Sink_Timer(self)
    sink_proess.run(self)


'''
-------------------------------------------------------------------------------------
---------------------------------- Communication not-sink------------------------------------
-------------------------------------------------------------------------------------
'''
# Event flag to signal that spanning_listener wants to write
listener_current_updated_irremovable = threading.Event()
listener_end_of_spanning = threading.Event()


def spanning_listener(self):
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
    while check_continuity_of_listening(self): 
        try:
            msg=retrieve_msg_from_buffer(listener_end_of_spanning)
            
            if msg.startswith(Emergecy_header.encode()) and msg.endswith(b'\n'):
                self.emergency_stop()
                break
            self.exchange_neighbors_info_communication(msg)
            # Message of building the path 
            if msg.startswith(Spanning_header.encode()) and msg.endswith(b'\n'):
                id_rec, data= decode_target_message(msg)       
                if id_rec == self.id :
                    if data==0:
                        if self.get_state()== Border: 
                            self.change_state_to(Irremovable_boarder)
                            write_log_message("Become Irremovable_boarder")
                        else: 
                            self.change_state_to(Irremovable)
                            write_log_message("Become Irremovable")
                        # Send message to the neighbors to inform the new changes 
                        msg= build_target_message(self.id)
                        send_msg(msg)
                        listener_current_updated_irremovable.set() # Flag that the state was changed to irremovable
                    elif data == Border_sink_confirm:
                        # It's a meaage flow from border to sink to verfiry the path and end the pahse
                        # verfiy if became already irremovable drone (part of the path)
                        if (self.get_state() == Irremovable):
                            forward_confirm_msg(self,Border_sink_confirm)
                elif id_rec == spanning_terminator: # Message sent by the sink announcing the end of spanning phase
                    end_msg= build_target_message(spanning_terminator)
                    send_msg(end_msg)
                    listener_end_of_spanning.set()
                    
                else: # Recieved msg refer to changes in state to irrremovable in one of the nighbors
                    self.update_state_in_neighbors_list(id_rec, Irremovable) 
            
            elif  msg.startswith(Info_header.encode()) and msg.endswith(b'\n'):
                    self.forward_data_message(msg, self.drone_id_to_sink[0])
            else: 
                continue
        except:
            write_log_message("Thread spanning_listener Interrupt received, stopping...")
            self.emergency_stop()   


'''
-------------------------------------------------------------------------------------
------------------------------------ Build path -------------------------------------
-------------------------------------------------------------------------------------
'''
# since the 6 neighbors has 2 different regions one close to sink and one in opposit 
# check the 3 neighbors that are close to the sink to know if any is irremovable
def find_close_neigboor_2sink(self):
    neighbor_irremovable= None
    filtered_neighbors= None
    # The search should be only in the neighbor_list but since it contains also the data of s0 (current spot)
    # But because we need to find min of occupied distance, we should remove the s0 from the list 
    # Sort the neighbor_list based on the distance
    reference_distance = self.get_neighbor_list()[0]['distance']
    # Filter neighbors with distance from sink less than current spot and sort them
    sorted_neighbors = sorted(
    [neighbor for neighbor in self.get_neighbor_list()[1:] if neighbor['distance'] < reference_distance],
    key=lambda x: x['distance'])
    # Extract the first 3 elements with the minimum distances
    three_min_neighbors = sorted_neighbors[:3]
    # Filter the neighbor_list with minimum distance the spot should be occupied "drones_in" > 0  
    filtered_neighbors = [neighbor for neighbor in three_min_neighbors if neighbor["drones_in"] > 0]
    if filtered_neighbors is not None:  # There are occupied neighbors
        # Find the neighbor with id of the drone that is irremovable 
        # if you arrive to irremovable 
        # Here is changed because the filtered_neighbors is a list 
        neighbor_irremovable = next((neighbor for neighbor in filtered_neighbors if Irremovable in neighbor['states']) , None)
        # No irremovable found 
        if neighbor_irremovable== None:
            return int((min(filtered_neighbors, key=lambda x: x["distance"])["drones_in_id"])[0]) # retuen the id of the drone 
        else: 
            # irremovable found, save it
            append_id_to_path( self.drone_id_to_sink, neighbor_irremovable["drones_in_id"] )
            return -1
    else: 
        # The case of no occupied neighbors close to sink around is very possible after further expansion
        # Check if any of the neighbors drones are preivious border 
        drone_previous_border = [neighbor for neighbor in self.get_neighbor_list() if ( neighbor["drones_in"] > 0 and neighbor["previous_state"]== Border or neighbor["previous_state"]== Irremovable_boarder ) ] 
        # If there are many drones had a state border befor chose the one closer to sink 
        if drone_previous_border>1: 
            return int((min(drone_previous_border, key=lambda x: x["distance"])["drones_in_id"])[0])
        elif drone_previous_border==1:
            return (drone_previous_border["drones_in_id"])[0]
        # Founding no drone it is not possible, after the further exapnsion, the drones will be between the old border an dthe new one 


def find_close_neigboor_2border(self):
    neighbor_irremovable= None 
    reference_distance = self.get_neighbor_list()[0]['distance']
    # Filter neighbors with distance from sink bigger than current spot and sort them
    sorted_neighbors = sorted(
    [neighbor for neighbor in  self.get_neighbor_list()[1:] if neighbor['distance'] > reference_distance],
    key=lambda x: x['distance'])
    # Extract the first 3 elements with the max distances (close to the border)
    three_max_neighbors = sorted_neighbors[:3]
    # Filter the neighbor_list for objects with "drones_in" > 0 as it contains a drone in 
    filtered_neighbors = [neighbor for neighbor in three_max_neighbors if neighbor["drones_in"] > 0]
    # Find the neighbor with drone that is irremovable or irremovable- border because 
    # if there is a irremovable or irremovable-border there is no need to send message
    # Here a mutex needed in this operation to read  the state
    neighbor_irremovable = next((neighbor for neighbor in filtered_neighbors if ( Irremovable in neighbor['states'] or Irremovable_boarder in neighbor['states']) ), None)
    if neighbor_irremovable== None: # No irremovable found, check what is the closest to the sink
        return int((max(filtered_neighbors, key=lambda x: x["distance"])["drones_in_id"])[0]) # Retuen the id of the drone 
    else: # There is a irremovable drone in occupied neighbors but usually it is only one
        append_id_to_path(self.drone_id_to_border, neighbor_irremovable["drones_in_id"])
        return -1 
  
def build_path(self):
    # Irremovable_boarder and Irremovable needs to build bath to the sink 
    target_id= find_close_neigboor_2sink(self) 
    if target_id != -1 : # No irremovable around send msg to a drone close to sink to make it irremovable 
        # save the id of the drone for future use to connect to sink
        append_id_to_path(self.drone_id_to_sink, target_id)
        msg= build_target_message(target_id)
        send_msg(msg)
    
    if self.get_state() != Irremovable_boarder:  # it is irrmovable doesnt belong to boarder no need to check (self.state != " irremovable- border" ) 
        target_id= find_close_neigboor_2border(self)  # since it doesnt belong to border then find to path to border 
        if target_id != -1 : 
            append_id_to_path(self.drone_id_to_border, target_id)
            msg= build_target_message(target_id)
            send_msg(msg)
    

'''
-------------------------------------------------------------------------------------
----------------------------------- Main function ----------------------------------
-------------------------------------------------------------------------------------
'''     
def spanning(self, vehicle): 
    write_log_message(" -------- Spanning -------- ")
    if self.id==1 and self.get_current_spot()["distance"]==0 : # if the drone is sink ( leader of the termination of the spaning phase)
        write_log_message(" -------- Spanning Sink-------- ")
        spanning_sink(self)
    
    else:
        xbee_thread = threading.Thread(target=spanning_listener, args=(self,))
        xbee_thread.start()

        # Update neigboors info after the end of expansion and wait the data to be recived
        # Needed to find what neigbour that became irremvable due to finding target 
        self.demand_neighbors_info()
        # Free drone wait for msg to become irremovable by another drone or wait broadcast from sink of finihing Spainning
        if (self.get_state()== Free) or self.get_state()== Border:
            write_log_message(" -------- Spanning Free or Border -------- ")
            # Wait for spanning_listener to signal that state has been changed ( doesn't keep the CPU busy.)
            while (not listener_current_updated_irremovable.is_set()) and ( not listener_end_of_spanning.is_set()):
                time.sleep(0.2)
            listener_current_updated_irremovable.clear() # need to be cleared for the next spanning
            # Here if the drone became part of the path the second if statment will be true and it will find path to sink-border
        
        # For Irremovables and Free drones that were changed  
        if (self.get_state()== Irremovable) or (self.get_state() == Irremovable_boarder):
            write_log_message(" -------- Spanning Irremovable or Irremovable_boarder -------- ")
            self.demand_neighbors_info()
            build_path(self)
            # Time needed so the drone in the sink direction received msg,  changed its state and try to build its path 
            time.sleep(2) 
            # Send message to the sink about the corrdinates 
            # Not all Irremovable found targets, irremovable can be only part of the path
            if self.target_detected and len(self.drone_id_to_sink)>0:
                self.send_data_message_station(vehicle, id_to_send_to= self.drone_id_to_sink[0]) 

        # Send a message that will travel from border to sink and that will annouce end of the pahse 
        if self.get_state()== Irremovable_boarder:
            while not self.drone_id_to_sink: # wait until list not empty 
                time.sleep(1)
            forward_confirm_msg(self,Border_sink_confirm)

        listener_end_of_spanning.wait() 
        listener_end_of_spanning.clear()
        xbee_thread.join() 
            
        