from .VESPA_module import *
from .form_border_one_direction import confirm_border_connectivity, re_form_border, circulate_msg_along_border, build_border_message, decode_border_message, forward_broadcast_message

set_env(globals())

'''
-------------------------------------------------------------------------------------
---------------------------------- Communication ------------------------------------
-------------------------------------------------------------------------------------
'''
terminator_indecator=-1
def build_local_movement_message(target_id, destination):
    max_byte_count = determine_max_byte_size(target_id)
    # Start the message with the header and the max byte count for target_id
    message = Local_balance_header.encode() + struct.pack('>B', max_byte_count)
    # Add the target_id to the message using the determined byte count
    message += target_id.to_bytes(max_byte_count, 'big')
    # Append the destination to the message ( 1 byte becausee it is spot 0-6 )
    message += destination.to_bytes(1, 'big')
    message += b'\n'
    return message

def decode_local_movement_message(message):
    max_byte_count = struct.unpack('>B', message[1:2])[0]  # Dereferenced the tuple to get the value
    # Extract target_id using max byte count
    target_id_start = 2
    target_id_end = target_id_start + max_byte_count
    target_id = int.from_bytes(message[target_id_start:target_id_end], 'big')
    # Extract the destination
    destination = int.from_bytes(message[target_id_end:target_id_end+1], 'big')  # Since destination is always one byte
    return target_id, destination

# Find what spot a drone should move along the border to achieve the balance with a difference of 1 between neighbors' border spots
def lead_local_balancing(self):
    No_free_drone=None 
    # Extract the 'free' drone IDs in s0
    s0 = next(spot for spot in self.self.get_neighbor_list() if spot["name"] == "s0")
    s0_free_ids = sorted([s0['drones_in_id'][idx] for idx, state in enumerate(s0["states"]) if state == Free])
    if s0_free_ids==0:
        No_free_drone=1 

    # Identify all spot with the "border" state but out of the s0 (the current spot),  List of neighbors without S0 
    border_spots = [spot for spot in  self.get_neighbor_list()[1:] if (Border or Irremovable_boarder) in spot["states"]]
    moves = []
    # For each "neighbor border" spot, calculate the difference in the number of "free" states with s0
    for spot in border_spots:
        spot_free_count = sum(1 for state in spot["states"] if state == Free)
        # Track if the S0 and the border neigbor are empty means partially there are no more free drone 
        if No_free_drone != None and spot_free_count==0:
            No_free_drone+=1            
        diff = len(s0_free_ids) - spot_free_count
        # If s0 has more 'free' drones than the "border" spot by more than 1, move the smallest "free" drone ID
        while diff > 1 and s0_free_ids:
            drone_id_to_move = s0_free_ids.pop(0)  # Get the smallest "free" drone ID
            destination_number = spot["name"][1:]  # Extract the number after "s"
            moves.append((drone_id_to_move, destination_number)) # append tuple of id , destination 
            diff -= 1

    if No_free_drone == len(border_spots)+1: # If no drone found in the border neighbors and also the current border  
        moves=-1 
    return moves

def build_shared_allowed_spots_message(self):
    # Extract data of S0 current spot
    s0 = next(spot for spot in  self.get_neighbor_list() if spot["name"] == "s0")
    # Create a list to store the IDs of drones with 'free' state in s0
    targets_id = [drone_id for drone_id, state in zip(s0["drones_in_id"], s0["states"]) if state == Free]
    max_byte_count_targets = max(self.determine_max_byte_size(num) for num in targets_id)
    # Start message with 'G', max byte count for targets_id, followed by the length of targets_id
    message = Guidance_header.encode() 
    message += struct.pack('>BB', max_byte_count_targets, len(targets_id))
    for num in targets_id:
        message += num.to_bytes(max_byte_count_targets, 'big')
    # Add length of allowed_spots
    message += struct.pack('>B', len(self.allowed_spots))
    for num in self.allowed_spots:
        message += num.to_bytes(1, 'big')
    message += b'\n'
    
    return message

def decode_shared_allowed_spots_message(message):
    max_byte_count_targets, targets_id_length = struct.unpack('>BB', message[1:3])
    # Determine the start and end indices for the target_id
    targets_id_indices = [(i * max_byte_count_targets + 3, (i + 1) * max_byte_count_targets + 3) for i in range(targets_id_length)]
    targets_id = [int.from_bytes(message[start:end], 'big') for start, end in targets_id_indices]
    allowed_spots_start = targets_id_indices[-1][1]
    allowed_spots_length = struct.unpack('>B', message[allowed_spots_start:allowed_spots_start + 1])[0]
    # Determine the start and end indices for the guidance numbers
    allowed_spots_indices = [(i + allowed_spots_start + 1, i + allowed_spots_start + 2) for i in range(allowed_spots_length)]
    # Extract the numbers in guidance_list and convert to string with "S" prefix
    allowed_spots = [int.from_bytes(message[start:end], 'big') for start, end in allowed_spots_indices]    
    return targets_id, allowed_spots

def check_continuity_of_listening(self):
    if not self.end_of_balancing.is_set():
        return True
    else: 
        return False
'''
-------------------------------------------------------------------------------------
---------------------------------- Border Procedure ---------------------------------
-------------------------------------------------------------------------------------
'''
def construct_allowed_spots(self):
    # Should be done using list constructed over the phases, thus any new populated area will be removed from the list ofallowed spots 
    spot_populated = [int(neighbor["name"][1:]) for neighbor in self.get_neighbor_list() if neighbor["drones_in"] > 0]
    if spot_populated: 
        for spot in spot_populated:
            self.allowed_spots.remove(spot)

def share_allowed_spots_with_free(self):
    message= build_shared_allowed_spots_message(self)
    send_msg(message) 

def send_lead_local_balancing_message(all_moves):
    if all_moves != -1 : # there are moves to be taken 
        for move in all_moves:
            id, destination = move  # Unpack the tuple
            msg= build_local_movement_message(id, destination)
            send_msg(msg)
        time.sleep((a/speed_of_drone)+2)  

def rest_border_timer(border_t):
    with border_t.lock_border:
       border_t.remaining_time = border_t.timeout   

class Boarder_Timer:
    def __init__(border_t,self,timeout=(a/speed_of_drone)+10):
        border_t.timeout = timeout
        border_t.remaining_time = border_t.timeout
        border_t.lock_border= threading.Lock()  
        border_t.message_thread = threading.Thread(target=border_listener, args=(border_t,self,timeout,))
        border_t.message_thread.start()
        self.end_of_balancing= threading.Event()

    def run(border_t, self):
        while True: 
            with border_t.lock_border:  # Acquire the lock
                border_t.remaining_time -= 0.1
                write_log_message(f"Remaining time for border_t.lock : {border_t.remaining_time:.2f} seconds")
                if border_t.remaining_time <= 0:
                    border_t.time_up(border_t,self)
                    break
            time.sleep(0.1)               
        
    def time_up(border_t,self):
        # Called when the timer reaches its timeout without being reset, meanning no more movment is done
        write_log_message("Time's up no more movement detected ! ")
        # Ensure that balanced achived 
        self.demand_neighbors_info()
        all_moves= lead_local_balancing(self)
        if all_moves==-1: # No Free drones around, then possible end of the algorithm 
           border_t.local_balancing.set() # If no drones around that still means local balancing, so both messages will be sent
           circulate_msg_along_border(Algorithm_termination_header)
           circulate_msg_along_border(Balance_header) 
        else: 
            # Keep checking until all dones is distributed correctly
            while len(all_moves) >0 : # there are drones need to be moved 
                send_lead_local_balancing_message( all_moves)
                time.sleep( border_t.timeout )
                all_moves= lead_local_balancing(self)

            # Moves =0, meaning local balance is achived
            border_t.local_balancing.set() # Flag to identify local balancing
            # Allowed spots need to be sent in this stage,the thread of listining of free drone would joined after completing the circle 
            share_allowed_spots_with_free(self)
            circulate_msg_along_border(Balance_header)

        self.end_of_balancing.wait()
        border_t.local_balancing.clear()
        self.end_of_balancing.clear() 
        border_listener.join() # stop listenning

def border_listener(self,border_t):
    
    while check_continuity_of_listening(self): # the end is not reached , keep listenning 
        try: 

            msg= self.retrieve_msg_from_buffer(self.end_of_balancing) 
            
            self.exchange_neighbors_info_communication(msg)

            if msg.startswith(Emergecy_header.encode()) and msg.endswith(b'\n'):
                self.emergency_stop()
                break
            
            elif msg.startswith(Arrival_header.encode()) and msg.endswith(b'\n'):
                rest_border_timer(border_t) # New arrival reset the timer 
                # This can be recived in case of drone arrive to the current spot or another border neigbors
                positionX, positionY, state, id_value= self.decode_spot_info_message(msg)
                self.update_neighbors_list(positionX, positionY, state, id_value )   
                all_moves= lead_local_balancing(self)
                send_lead_local_balancing_message(self, all_moves)

            elif ( msg.startswith(Balance_header.encode()) or msg.startswith(Algorithm_termination_header.encode()) ) and msg.endswith(b'\n') :
                
                if msg.startswith(Balance_header.encode()):
                    header_in_use= Balance_header
                else:
                    header_in_use= Algorithm_termination_header

                sender_id, target_id, candidate= decode_border_message(msg)
                # The border drone will respond to the message only if it is in local balancing otherwise drop it 
                if border_t.local_balancing.is_set():
                    # End of the balancing broadcast msg
                    if target_id== terminator_indecator:                         
                        # Here any drone in any state needs to forward the boradcast message and rise ending flag  
                        forward_broadcast_message(self, header_in_use,candidate)
                        if header_in_use== Algorithm_termination_header:
                                self.VESPA_termination.set()
                        # This nedd to be set in both cases so the pahse finish to recognize the end of the algorithm 
                        self.end_of_balancing.set()
                    
                    elif self.id == target_id: # Drone respond only if it is targeted
                        if candidate == self.id and (self.get_state()== Border or self.get_state()== Irremovable_boarder):# the message recived contains the id of the drone means the message came back  
                            Broadcast_Msg= build_border_message(self,header_in_use, terminator_indecator, self.id)
                            send_msg(Broadcast_Msg) # Bordacst doesnt need to be waiting conformation 
                            if header_in_use== Algorithm_termination_header:
                                self.VESPA_termination.set()
                            self.end_of_balancing.set() 
                            
                        else: # The current drone received a message from a candidate border so it needs to forward it                                      
                            if self.get_state()== Border or self.get_state()== Irremovable_boarder:
                                if self.current_target_id is not None:
                                    msg= build_border_message(self,header_in_use, self.current_target_id, candidate)
                                    send_msg(msg) 
                                
                    else: # Drone is not targeted ( doesnt matter if it is free or candidate) thus it drops the message 
                        continue
        except:
            write_log_message("Thread border_listener Interrupt received, stopping...")
            self.emergency_stop() 
                
'''
-------------------------------------------------------------------------------------
------------------------------- Free drone Procedure --------------------------------
-------------------------------------------------------------------------------------
'''
def communication_balancing_free_drones(self,vehicle):
    
    while check_continuity_of_listening(self): # the end is not reached , keep listenning 
        try: 

            msg= self.retrieve_msg_from_buffer(self.end_of_balancing) 
            
            self.exchange_neighbors_info_communication(msg)

            if msg.startswith(Emergecy_header.encode()) and msg.endswith(b'\n'):
                self.emergency_stop()
                break
                        
            elif msg.startswith(Arrival_header.encode()) and msg.endswith(b'\n'):
                # This can be recived in case of drone arrive to the current spot or another border neigbors
                positionX, positionY, state, id_value= self.decode_spot_info_message(msg)
                self.update_neighbors_list(positionX, positionY, state, id_value )

            # Recieve message from the border to move 
            elif msg.startswith(Local_balance_header.encode()) and msg.endswith(b'\n'):
                rec_id,destination= self.decode_local_movement_message(msg)
                if rec_id == self.id: # Current drone is targeted
                    self.move_to_spot(vehicle, destination)
                    data_msg= self.build_spot_info_message(Arrival_header)
                    send_msg(data_msg)

            elif msg.startswith(Guidance_header.encode()) and msg.endswith(b'\n'):
                targets_id, allowed_spots= decode_shared_allowed_spots_message(msg)
                if self.id in targets_id:
                    self.allowed_spots= allowed_spots # Assign the allowed_spots of the spot of border to current drone so will not go back
            
            # It is needed in case of reciving the message of ending 
            elif msg.startswith(Balance_header.encode()) and msg.endswith(b'\n'):
                sender_id, target_id, candidate= decode_border_message(msg)
                if target_id==terminator_indecator:                         
                    # Free drones doesnt forward they just rise the flag 
                    self.end_of_balancing.set()
                else: # nothing to do if it is not broadcast 
                    continue
        except:
            write_log_message("Thread communication_balancing_free_drones Interrupt received, stopping...")
            self.emergency_stop() 

def search_to_border(self):
    border_found=False
    
    #Find states with 'border'
    border_states = [s for s in self.get_neighbor_list() if (Border or Irremovable_boarder) in s["states"]]
    
    # If only one border state is found
    if len(border_states) == 1:
        border_found= True
        return border_found, int(border_states[0]["name"][1:]) # Return only the number of spot 
    
    # If multiple border states are found
    elif len(border_states) > 1:
        # Sort by drones_in
        border_found= True
        border_states.sort(key=lambda x: x["drones_in"])
        
        # Filter states with the least drones
        least_drones = border_states[0]["drones_in"]
        least_drones_states = [s for s in border_states if s["drones_in"] == least_drones]
        
        # Choose randomly the states with the least drones if many have same numbers of drones in 
        chosen_spot= random.choice(least_drones_states)
        return border_found, int(chosen_spot["name"][1:])
    
    # If no border state is found, find the spot with the maximum distance (close to the border)
    else:
        max_distance_spot = max( self.get_neighbor_list(), key=lambda x: x["distance"])
        return border_found, int(max_distance_spot["name"][1:]) 



'''
-------------------------------------------------------------------------------------
------------------------------- General Procedure --------------------------------
-------------------------------------------------------------------------------------
'''

def balancing(self, vehicle):
    write_log_message(" -------- Balancing -------- ")
    # Confirm the border first if it doesnt exit reform it 
    border_well_confirmed= confirm_border_connectivity(self)
    if not border_well_confirmed: 
        re_form_border(self) 

    # Border is the chef of the spot 
    if self.get_state()== Border or Irremovable_boarder:
        write_log_message(" -------- Balancing Border or Irremovable_boarde -------- ")
        construct_allowed_spots(self) # Create the allwoed spots
        border_process=Boarder_Timer()
        border_process.run()
    
    elif self.get_state() == Free: 
        write_log_message(" -------- Balancing Free -------- ")
        xbee_receive_message_thread = threading.Thread(target=communication_balancing_free_drones, args=(self,vehicle,)) #pass the function reference and arguments separately to the Thread constructor.
        xbee_receive_message_thread.start()
        border_found= False
        while border_found == False : # no border in the nieghbor
            self.demand_neighbors_info()
            border_found, spot_to_go = search_to_border(self)
            # Move             
            self.move_to_spot(vehicle, spot_to_go)
        # This message will inform the border drone around and its neighbors
        data_msg= self.build_spot_info_message(Arrival_header)
        send_msg(data_msg)
        self.end_of_balancing.wait()
        self.end_of_balancing.clear() 
    
    xbee_receive_message_thread.join()