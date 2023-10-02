from expan import *

class Boarder_Timer:
    def __init__(border_t,self,timeout=2*movement_time):
        border_t.timeout = timeout
        border_t.remaining_time = border_t.timeout
        #border_t.lock_sink = threading.Lock()  # Create a lock
        border_t.message_thread = threading.Thread(target=border_listener, args=(border_t,self,timeout,))
        border_t.message_thread.start()
        border_t.end_of_balancing= threading.Event()

    def run(border_t, self):
        while True:
            with border_t.lock_sink:  # Acquire the lock
                border_t.remaining_time -= 0.5
                print(f"Remaining time: {border_t.remaining_time:.2f} seconds")
                if border_t.remaining_time <= 0:
                    border_t.time_up(border_t,self)
                    break
            time.sleep(0.5)

    def time_up(border_t,self):
        #Called when the timer reaches its timeout without being reset.
        print("Time's up! ")
        
        border_t.local_balancing.set() # flag to tell listener that it is the end
        
        # border_listener.join() # stop listenning 


def border_listener(self,border_t, timeout):
    while not border_t.end_of_balancing.is_set(): # the end is not reached , keep listenning 
        msg= self.retrive_msg_from_buffer() 
        
        # if a drone asked the info it shouls be send, but no need to recive respond 
        if msg == Demand_header.encode() + b'\n':
            data_msg= self.build_spot_info_message(Reponse_header) # Build message that contains all data
            self.send_msg(data_msg)

        if msg.startswith(Arrival_header.encode()) and msg.endswith(b'\n'):
            print("Message received!")
            # This can be recived in case of drone arrive to the current spot or another border neigbors
            positionX, positionY, state, id_value= self.decode_spot_info_message(msg)
            self.update_neighbors_list(positionX, positionY, state, id_value )
            border_t.remaining_time = border_t.timeout     
            id, destination= self.lead_local_balancing()
            self.build_local_movement_message(id, destination)




    border_t.end_of_balancing.clear() # reset so the next spanning the listenning loop will be activated  


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

def communication_balancing(self,vehicle):
    
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

    # Recieve message from the border to move 
    if msg.startswith(Local_balance_header.encode()) and msg.endswith(b'\n'):
        rec_id,destination= self.decode_local_movement_message(msg)
        if rec_id == self.id: # Current drone is targeted
            self.move_to_spot(vehicle, destination)
            data_msg= self.build_spot_info_message(Arrival_header)
            self.send_msg(data_msg)

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

def search_going_to_border(self):
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

# the border is the chef of the spot 
# end when a message from each border to another that balancig is done 

# This function saves the spots that are occupied regarless if it is in the dominated direction or not 
def save_occupied_spots(self):
    occupied_spots = [int(spot["name"][1:]) for spot in self.neighbor_list if spot["drones_in"] != 0]
    return occupied_spots

def construct_allowed_spots(self, border_neighbors ):
    # this should be done in balancing when the border all of it known
    # save occupied spot sould be only for the free drone or irremovable ( dont consider the border )
    self.border_neighbors = self.save_occupied_spots() # The border drone will save the occupied spot for the next expansion 


def balancing(self, vehicle):
    if self.state== Border:
        border_process=Boarder_Timer()
        border_process.run()

    
    if self.state == Free: 
        self.xbee_receive_message_thread = threading.Thread(target=self.communication_balancing, args=(self,vehicle,)) #pass the function reference and arguments separately to the Thread constructor.
        self.xbee_receive_message_thread.start()
        brder_found= False
        while border_found == False : # no border in the nieghbor
            self.build_data_demand_message()
            self.neighbors_list_updated.wait()
            self.neighbors_list_updated.clear()
            border_found, spot_to_go = search_going_to_border(self)
            # Move             
            self.move_to_spot(vehicle, spot_to_go)
        # This message will be read by the border drone and its niegbor
        data_msg= self.build_spot_info_message(Arrival_header)
        self.send_msg(data_msg)



