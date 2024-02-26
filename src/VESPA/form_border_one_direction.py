from .VESPA_module import *

set_env(globals())

'''
This module is used to send messages by sending message only to one drone by searching clockwise from empty spot 
'''

'''
-------------------------------------------------------------------------------------
---------------------------------- Communication ------------------------------------
-------------------------------------------------------------------------------------
'''
def build_border_message(self,header,target_ids, candidate):
    # Determine max byte count for numbers
    max_byte_count = max(
                        [determine_max_byte_size(num) for num in target_ids ]+
                        [determine_max_byte_size(candidate)]+
                        [determine_max_byte_size(id)]
                        )
    # Start message with 'F', followed by max byte count and then the length of the propagation_indicator
    message = header.encode() + struct.pack('>BB', max_byte_count, len(target_ids))

    message += struct.pack('>B',len(target_ids))
    for num in target_ids:
            message += num.to_bytes(max_byte_count, byteorder='big',signed=True)
    # Append the sender using the determined byte count
    message += self.id.to_bytes(max_byte_count, 'big')
    # Append the candidate using the determined byte count
    message += candidate.to_bytes(max_byte_count, 'big')
    message += b'\n'
    return message

def decode_border_message(message):
    # Read the header (assuming it's a fixed length -- you'll need to define this)
    header_length = 1  # Replace with the actual length of the header
    message = message[header_length:]

    # Read the max byte count and length of target_ids
    max_byte_count, num_target_ids = struct.unpack('>BB', message[:2])
    message = message[2:]

    # Remove the redundant target_ids length byte
    message = message[1:]

    # Read target ids based on the max_byte_count
    target_ids = []
    for _ in range(num_target_ids):
        num_bytes = message[:max_byte_count]
        target_id = int.from_bytes(num_bytes, 'big',signed=True)
        target_ids.append(target_id)
        message = message[max_byte_count:]

    # Read sender id
    num_bytes = message[:max_byte_count]
    sender_id = int.from_bytes(num_bytes, 'big')
    message = message[max_byte_count:]

    # Read candidate
    num_bytes = message[:max_byte_count]
    candidate = int.from_bytes(num_bytes, 'big')
    message = message[max_byte_count:]

    return sender_id, target_ids, candidate

def circle_completed(self):
        if check_border_candidate_eligibility(self):    
            # These cases should be considered for the 
            if self.get_state() == Irremovable:
                self.change_state_to(Irremovable_boarder)
            else: 
                self.change_state_to(Border) 
            
            Broadcast_Msg= build_border_message(self,Forming_border_header,[-1], self.id)
            #send_msg_border_upon_confirmation(self, Broadcast_Msg)
            send_msg(Broadcast_Msg)
            self.Forming_Border_Broadcast_REC.set() # to end the the loop
        else:
            # the drone got new neigbors and became Free            
            if self.get_state() == Owner: # the one for example a irremovable should not change and stay irremovabe 
                self.change_state_to(Free)

def border_broadcast_respond(self, candidate):
    if candidate in self.rec_candidate: # the sender of broadcast already sent msg to the current drone so it is part of the circle
        # re-check the the droen around still have same situation and still can be border
        if check_border_candidate_eligibility():
            if self.get_state() == Irremovable:
                self.change_state_to(Irremovable_boarder)
            else: 
                self.change_state_to(Border) 
    else: # if drone doesnt have the candidate or the sourounding has changed
         if self.get_state() == Owner: # the one for example a irremovable should not change and stay irremovabe 
                self.change_state_to(Free)


def forward_broadcast_message(self,header,candidate):
    '''
    Build and send messgaes to all the niegbors and since only it is a broadcast
    then propagation_indicator, targets_ids both are [-1].
    The most important is the candidate that fired the broadcast, which is already recieved from neigbors
    Note: since the message will be sent to all the drone around , but rememeber the ones that already received
    it will not recieved it again and th reason is the flag that end the listener is raised and no reading of buffer will be performed
    '''
    msg= build_border_message(self, header,[-1],candidate) # as you see the candidate is resent as it was recived
    send_msg(msg)

def border_broadcast_respond(self, candidate):
    if candidate in self.rec_candidate: # the sender of broadcast already sent msg to the current drone so it is part of the circle
        # re-check the the droen around still have same situation and still can be border
        if check_border_candidate_eligibility(self):
            self.change_state_to(Border)
    else: # if drone doesnt have the candidate or the sourounding has changed
        self.change_state_to(Free)

def form_border_one_direction(self,header,msg):
    sender_id, target_ids, candidate= decode_border_message(msg)
    reset_timer_forme_border(self)

    if sender_id in self.current_target_ids and candidate in self.rec_candidate:
        # MESSAGR REC"
        with self.candidate_to_send_lock:
            if candidate in self.candidate_to_send:
                self.candidate_to_send.remove(candidate)

    if len(target_ids)==1 and target_ids[0]==-1:
        if self.border_candidate==True:
            border_broadcast_respond(self, candidate)
        # Here any drone in any state needs to forward the boradcast message and rise ending flag
        forward_broadcast_message(self, Forming_border_header,candidate)
        finish_timer_forme_border(self)
        self.Forming_Border_Broadcast_REC.set()

    if self.id in  target_ids  and target_ids :# targets exist not empty s
        if self.id == candidate:
            circle_completed(self)
            finish_timer_forme_border(self)

        else: 
            with self.candidate_to_send_lock:
                if candidate not in self.candidate_to_send:
                    self.candidate_to_send.append(candidate)


def send_msg_border_until_confirmation(self,header):
    while not self.Forming_Border_Broadcast_REC.is_set():
        
        # Copy messages_to_be_sent and iterate in it trying to send all the msg 
        # self.sending_messgae_list will change when a message is received the candidate will be pulled out 
        candidates_to_process = []
        with self.candidate_to_send_lock:
            if  self.candidate_to_send :
                candidates_to_process = list(self.candidate_to_send)
        
        if not self.Forming_Border_Broadcast_REC.is_set(): # dont reset at the end of phase since it listeners will be bloked
            self.demand_neighbors_info()
            check_border_candidate_eligibility(self)
            self.current_target_ids= choose_spot_right_handed(self)
      
        if self.border_candidate == True:
            for candidate in candidates_to_process: 
                if candidate not in self.rec_candidate:
                    self.rec_candidate.append(candidate)
                if self.Forming_Border_Broadcast_REC.is_set():
                    break
                if self.current_target_ids is not None:
                    msg= build_border_message(self,header,self.current_target_ids, candidate) 
                send_msg(msg)
                time.sleep(exchange_data_latency)# time untile the message arrives 
            time.sleep(exchange_data_latency*2)
            

''''
-------------------------------------------------------------------------------------
----------------------------------- Main functions ----------------------------------
-------------------------------------------------------------------------------------
'''

def reset_timer_forme_border(self):
    with self.lock_boder_timer:
        self.remaining_time_forme_border=30*exchange_data_latency

# called by other threads 
def finish_timer_forme_border(self):
    with self.lock_boder_timer:
        self.remaining_time_forme_border=0


def count_element_occurrences(self):
        # Find the maximum element in the direction_taken list
        max_element = 7  # s0 to s6
        # Create a dictionary to store the frequencies, similar to hash map to count occurrences
        frequency_dict = {i: 0 for i in range(max_element + 1)}
        # Count the occurrences of each element
        for direction in self.direction_taken:
            frequency_dict[direction] += 1
        # Find the maximum frequency
        max_freq = max(frequency_dict.values())
        max_indices = [key for key, value in frequency_dict.items() if value == max_freq]
        # If there is only one occurrence of the maximum frequency, return its index
        if len(max_indices) == 1:
            return max_indices[0]
        # If there are multiple occurrences of the maximum frequency, choose randomly
        return random.choice(max_indices)
    
def check_border_candidate_eligibility(self):
    if self.get_state() != Owner or self.get_state() != Irremovable:
        self.border_candidate=False
        return self.border_candidate
    
    self.border_candidate=False

    unoccupied_spots_counter = 0
    for neighbor in self.get_neighbor_list():
        # This is in the further expansion is needed , where the candidate is decides based on the allowed_spots
        if self.get_previous_state()==Border or self.get_previous_state()==Irremovable_boarder:
            if int(neighbor["name"][1:]) in self.allowed_spots:
                if neighbor["drones_in"] == 0:
                    unoccupied_spots_counter += 1               
        else: # the drone was not part of the previous border
           if neighbor["drones_in"] == 0: # spot also is not occupied
               unoccupied_spots_counter += 1

    if unoccupied_spots_counter>0 and self.get_current_spot() ["drones_in"]==1 and (self.get_state()==Owner or self.get_state()==Irremovable ) : # at least one spot is empty so the drone can be part of he border
        if  self.all_neighbor_spots_owned(): 
            self.border_candidate=True
    else: 
        self.border_candidate=False
        
    return self.border_candidate

def choose_spot_right_handed(self):
    neighbor_list_x = self.get_neighbor_list()[1:]
    n = len(neighbor_list_x)
    first_empty_index = None
    # Find the first empty zone
    for i, neighbor in enumerate(neighbor_list_x):
        if neighbor["drones_in"] == 0:
            first_empty_index = i
            break
    # If no empty zone is found, return None
    if first_empty_index is None:
        return None
    # Search for the next drone in a circular fashion
    for j in range(1, n+1):
        next_index = (first_empty_index + j) % n
        if neighbor_list_x[next_index]["drones_in"] > 0:
            return [neighbor_list_x[next_index]["drones_in_id"][0]]
    return None

def create_target_list(self, header):
        target_ids=[]
        if header==Balance_header: # Targets are only the border ones
            for s in self.get_neighbor_list():
                if 'border' in s['states']:
                    border_indices = [idx for idx, state in enumerate(s['states']) if state == 'border']
                    target_ids.extend([s['drones_in_id'][idx] for idx in border_indices])
        else:
            for s in self.get_neighbor_list():
                target_ids.extend(s["drones_in_id"]) # add the id of all the niegbors including the current
        return target_ids

def start_msg_one_direction(self):
    with self.candidate_to_send_lock:
        if self.id not in self.candidate_to_send:
            self.candidate_to_send.append(self.id)
            self.rec_candidate.append(self.id)

def reset_border_variables(self): 
    self.current_target_ids=[]
    self.candidate_to_send=[]
    self.rec_candidate=[]
    self.direction_taken=[] 
    self.border_candidate=False
