from expan import *

class Boarder_Timer:
    def __init__(border_t,self,timeout=2*movement_time):
        border_t.timeout = timeout
        border_t.remaining_time = border_t.timeout
        border_t.message_thread = threading.Thread(target=border_listener, args=(border_t,self,timeout,))
        border_t.message_thread.start()
        self.end_of_balancing= threading.Event()

    def run(border_t, self):
        while not self.end_of_balancing.is_set():
            while True: 
                with border_t.lock_sink:  # Acquire the lock
                    border_t.remaining_time -= 0.5
                    print(f"Remaining time: {border_t.remaining_time:.2f} seconds")
                    if border_t.remaining_time <= 0:
                        border_t.time_up(border_t,self)
                        break
                time.sleep(0.5)               
        border_listener.join() # stop listenning
        self.end_of_balancing.clear() 

    def time_up(border_t,self):
        #Called when the timer reaches its timeout without being reset.
        print("Time's up! ")
        border_t.local_balancing.set() # flag to tell identify local balancing
        message= self. build_shared_allowed_spots_message()
        self.send_msg(message) 
        self.Fire_border_msg(Balance_header)

'''---------------------------------- Communication Border drone------------------------------------'''
def border_listener(self,border_t, timeout):
    while not self.end_of_balancing.is_set(): # the end is not reached , keep listenning 
        
        msg= self.retrive_msg_from_buffer() 
        
        # if a drone asked the info it shouls be send, but no need to recive respond 
        if msg == Demand_header.encode() + b'\n':
            data_msg= self.build_spot_info_message(Reponse_header) # Build message that contains all data
            self.send_msg(data_msg)

        if msg.startswith(Arrival_header.encode()) and msg.endswith(b'\n'):
            # This can be recived in case of drone arrive to the current spot or another border neigbors
            positionX, positionY, state, id_value= self.decode_spot_info_message(msg)
            self.update_neighbors_list(positionX, positionY, state, id_value )
            border_t.remaining_time = border_t.timeout     
            id, destination= self.lead_local_balancing()
            self.build_local_movement_message(id, destination)

        if msg.startswith(Balance_header.encode()) and msg.endswith(b'\n'):
            
            rec_propagation_indicator, target_ids, sender, candidate= self.decode_border_message(msg) 
            
            # The border drone will respon to the message only if it is in local balancing otherwise drop it 
            if border_t.local_balancing.is_set():

                # End of the balancing broadcast msg
                if len(target_ids)==1 and target_ids[0]==-1 and rec_propagation_indicator[0]==-1 :                         
                    # Here any drone in any state needs to forward the boradcast message and rise ending flag  
                    self.forward_broadcast_message(Balance_header,candidate) 
                    self.end_of_balancing.set()
                
                elif self.id in  target_ids: # the drone respond only if it is targeted
                    if candidate == self.id: # the message recived contains the id of the drone means the message came back  
                        Broadcast_Msg= self.build_border_message(Balance_header,[-1] ,[-1], self.id)
                        self.send_msg(Broadcast_Msg)
                        self.end_of_balancing.set() 
                        
                    else: # the current drone received a message from a candidate border so it needs to forward it          
                        if sender not in self.rec_propagation_indicator: 
                            if candidate not in self.rec_candidate:
                                self.rec_candidate.append(candidate) # add the received id to the list so when a Broadcast from the same id is recicved that means a full circle include the current drone is completed
                            self.rec_propagation_indicator= rec_propagation_indicator # change the propagation_indicator means message from opposite direction has arrived
                            self.forward_border_message(Balance_header, rec_propagation_indicator, target_ids, candidate) 

                else: # Drone is not targeted ( doesnt matter it it is free or candidate) thus it drops the message 
                        # Do anything but wait for end of the expansion broadcast
                        continue
                
def build_local_movement_message(self, target_id, destination):

    # Determine max byte count for numbers
    max_byte_count = self.determine_max_byte_size(target_id)
    
    # Start the message with the header and the max byte count for target_id
    message = Local_balance_header.encode() + struct.pack('>B', max_byte_count)
    
    # Add the target_id to the message using the determined byte count
    message += target_id.to_bytes(max_byte_count, 'big')
    
    # Append the destination to the message ( 1 bute becausee it is spot 0-6 )
    message += destination.to_bytes(1, 'big')
    
    # End with '\n'
    message += b'\n'
    
    return message

def decode_local_movement_message(self, message):

    # Extract max byte count for target_id
    max_byte_count = struct.unpack('>B', message[1:2])[0]  # Dereferenced the tuple to get the value
    
    # Extract target_id using max byte count
    target_id_start = 2
    target_id_end = target_id_start + max_byte_count
    target_id = int.from_bytes(message[target_id_start:target_id_end], 'big')
    
    # Extract the destination
    destination = int.from_bytes(message[target_id_end:target_id_end+1], 'big')  # Since destination is always one byte
    
    return target_id, destination

def lead_local_balancing(self):
    # Extract the 'free' drone IDs in s0
    s0 = next(station for station in self.neighbor_list if station['name'] == 's0')
    s0_free_ids = sorted([s0['drones_in_id'][idx] for idx, state in enumerate(s0['states']) if state == Free])
    
    # Identify all stations with the "border" state
    border_stations = [station for station in  self.neighbor_list if Border or Irremovable_boarder in station['states']]
    
    moves = []
    
    # For each "border" station, calculate the difference in the number of "free" states with s0
    for station in border_stations:
        station_free_count = sum(1 for state in station['states'] if state == Free)
        diff = len(s0_free_ids) - station_free_count
        
        # If s0 has more 'free' drones than the "border" station by more than 1, move the smallest "free" drone ID
        while diff > 1 and s0_free_ids:
            drone_id_to_move = s0_free_ids.pop(0)  # Get the smallest "free" drone ID
            destination_number = station['name'][1:]  # Extract the number after "s"
            moves.append((drone_id_to_move, destination_number))
            diff -= 1
    
    return moves

def build_shared_allowed_spots_message(self):
    
    # Extract data of S0 current spot
    s0 = next(station for station in self.neighbor_list if station['name'] == 's0')

    # Create a list to store the IDs of drones with 'free' state in s0
    targets_id = [drone_id for drone_id, state in zip(s0['drones_in_id'], s0['states']) if state == Free]

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
    # Extract max byte count for target_id and its length
    max_byte_count_targets, targets_id_length = struct.unpack('>BB', message[1:3])
    
    # Determine the start and end indices for the target_id
    targets_id_indices = [(i * max_byte_count_targets + 3, (i + 1) * max_byte_count_targets + 3) for i in range(targets_id_length)]
    
    # Extract target_id from message
    targets_id = [int.from_bytes(message[start:end], 'big') for start, end in targets_id_indices]
    
    # Extract the length of nums (guidance list)
    allowed_spots_start = targets_id_indices[-1][1]
    allowed_spots_length = struct.unpack('>B', message[allowed_spots_start:allowed_spots_start + 1])[0]
    
    # Determine the start and end indices for the guidance numbers
    allowed_spots_indices = [(i + allowed_spots_start + 1, i + allowed_spots_start + 2) for i in range(allowed_spots_length)]
    
    # Extract the numbers in guidance_list and convert to string with "S" prefix
    allowed_spots = ["S" + str(int.from_bytes(message[start:end], 'big')) for start, end in allowed_spots_indices]
    
    return targets_id, allowed_spots

'''---------------------------------- Communication Free drone-------------------------------------'''
def communication_balancing_free_drones(self,vehicle):
    while not self.end_of_balancing.is_set(): # the end is not reached , keep listenning 
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

        if msg.startswith(Arrival_header.encode()) and msg.endswith(b'\n'):
            # This can be recived in case of drone arrive to the current spot or another border neigbors
            positionX, positionY, state, id_value= self.decode_spot_info_message(msg)
            self.update_neighbors_list(positionX, positionY, state, id_value )
            # No neighbors_list_updated.set() is needed here because that should be raised only after adking data 

        # Recieve message from the border to move 
        if msg.startswith(Local_balance_header.encode()) and msg.endswith(b'\n'):
            rec_id,destination= self.decode_local_movement_message(msg)
            if rec_id == self.id: # Current drone is targeted
                self.move_to_spot(vehicle, destination)
                data_msg= self.build_spot_info_message(Arrival_header)
                self.send_msg(data_msg)

        if msg.startswith(Guidance_header.encode()) and msg.endswith(b'\n'):
            targets_id, allowed_spots= decode_shared_allowed_spots_message(msg)
            if self.id in targets_id:
                self.allowed_spots= allowed_spots # Assign the allowed_spots of the spot of border to current drone so will not go back
        
        # It is needed in case of reciving the message of ending 
        if msg.startswith(Balance_header.encode()) and msg.endswith(b'\n'):
            rec_propagation_indicator, target_ids, sender, candidate= self.decode_border_message(msg)
            if len(target_ids)==1 and target_ids[0]==-1 and rec_propagation_indicator[0]==-1 :                         
                # Free drones doesnt forward they just rise the flag 
                self.end_of_balancing.set()
            else: # nothing to do if it is not broadcast 
                continue

'''------------------------------------- Movement Free drone --------------------------------------'''
def search_to_border(self):
    border_found=False
    
    #Find states with 'border'
    border_states = [s for s in self.neighbor_list if 'border' in s['states']]
    
    # If only one border state is found
    if len(border_states) == 1:
        border_found= True
        return border_found, int(border_states[0]['name'][1:])
    
    # If multiple border states are found
    elif len(border_states) > 1:
        # Sort by drones_in
        border_found= True
        border_states.sort(key=lambda x: x['drones_in'])
        
        # Filter states with the least drones
        least_drones = border_states[0]['drones_in']
        least_drones_states = [s for s in border_states if s['drones_in'] == least_drones]
        
        # Choose randomly the states with the least drones if many have same numbers of drones in 
        chosen_spot= random.choice(least_drones_states)
        return border_found, int(chosen_spot['name'][1:])
    
    # If no border state is found, find the spot with the maximum distance (close to the border)
    else:
        max_distance_spot = max(self.neighbor_list, key=lambda x: x['distance'])
        return border_found, int(max_distance_spot['name'][1:]) 

def balancing(self, vehicle):
    # Border is the chef of the spot 
    if self.state== Border or Irremovable_boarder:
        hover(vehicle) # hover while all the commuinication and balancing done 
        border_process=Boarder_Timer()
        border_process.run()
        set_to_move(vehicle)

    
    if self.state == Free: 
        self.xbee_receive_message_thread = threading.Thread(target=self.communication_balancing, args=(self,vehicle,)) #pass the function reference and arguments separately to the Thread constructor.
        self.xbee_receive_message_thread.start()
        brder_found= False
        while border_found == False : # no border in the nieghbor
            self.build_data_demand_message()
            self.neighbors_list_updated.wait()
            self.neighbors_list_updated.clear()
            border_found, spot_to_go = search_to_border(self)
            # Move             
            self.move_to_spot(vehicle, spot_to_go)
        # This message will be read by the border drone and its niegbor
        data_msg= self.build_spot_info_message(Arrival_header)
        self.send_msg(data_msg)



