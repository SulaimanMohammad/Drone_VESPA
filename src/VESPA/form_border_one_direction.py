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
        if self.border_candidate:    
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
        if self.border_candidate:
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

def form_border_one_direction(self,header,msg):
    if not self.Forming_Border_Broadcast_REC.is_set(): # React only if border is not formed yet 
        sender_id, target_ids, candidate= decode_border_message(msg)
        reset_timer_forme_border(self,header) # Reset for any message even from out the region because that means the border is not yet formed 
        if sender_id in self.neighbors_ids: # Signal comes from the neighbor drone, dont consider messages out of the region 
            if sender_id in self.current_target_ids and candidate in self.rec_candidate:
                # MESSAGR REC, confirmed"
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
                    if sender_id in self.message_sent_for_border: # The mesage came backward not in circle
                        self.border_formed=False
                        finish_timer_forme_border(self)

                    else: 
                        circle_completed(self)
                        self.border_formed= True
                        finish_timer_forme_border(self)
                else: 
                    with self.candidate_to_send_lock:
                        if candidate not in self.candidate_to_send:
                            self.candidate_to_send.append(candidate)


def send_msg_border_until_confirmation(self,header):
    while not self.Forming_Border_Broadcast_REC.is_set() and (not self.expansion_stop.is_set()) and (not self.Emergency_stop.is_set()):

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
        time.sleep(exchange_data_latency)
            

def verify_border(self,header, msg):
    if not self.border_verified.is_set():
        sender_id, target_ids, candidate= decode_border_message(msg)
        reset_timer_forme_border(self, header)
        if sender_id in self.neighbors_ids: # Signal comes from the neighbor drone, dont consider messages out of the region 
            if len(target_ids)==1 and target_ids[0]==-1:
                # Here any drone in any state needs to forward the boradcast message and rise ending flag
                forward_broadcast_message(self, header,candidate)
                finish_timer_forme_border(self)
                self.border_verified.set()

            if self.id in  target_ids and target_ids :# targets exist not empty s
                if self.id == candidate and (self.get_state()== Border or self.get_state()== Irremovable_boarder) :
                    Broadcast_Msg= build_border_message(self,header,[-1], self.id)
                    #send_msg_border_upon_confirmation(self, Broadcast_Msg)
                    send_msg(Broadcast_Msg) # bordacst doent need to be waiting conformation 
                    finish_timer_forme_border(self)
                    self.border_verified.set() # to end the the loop
                else:
                    if self.get_state()== Border or self.get_state()== Irremovable_boarder:
                        self.current_target_ids= choose_spot_right_handed(self,self.neighbor_list_upon_border_formation )
                        msg= build_border_message(self,header,self.current_target_ids, candidate)
                        send_msg(msg) 

''''
-------------------------------------------------------------------------------------
----------------------------------- Main functions ----------------------------------
-------------------------------------------------------------------------------------
'''

def reset_timer_forme_border(self, header):
    with self.lock_boder_timer:
        if header== Forming_border_header:
            self.remaining_time_forme_border=60 # Contains waiting and confim the msg arrival 
        else:
            self.remaining_time_forme_border=30   # This used in case of verfiy the border the messages flow fast

# called by other threads 
def finish_timer_forme_border(self):
    with self.lock_boder_timer:
        self.remaining_time_forme_border=0
  
def check_border_candidate_eligibility(self):
    if (self.get_state() == Free): # Irremovable and border are also owner 
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

    if unoccupied_spots_counter>0 and (self.get_current_spot() ["drones_in"]==1) : # at least one spot is empty so the drone can be part of he border
        if  self.all_neighbor_spots_owned(): 
            self.border_candidate=True
    else: 
        self.border_candidate=False
        
    return self.border_candidate

def choose_spot_right_handed(self, neighbor_list_upon_border=None):
    
    if neighbor_list_upon_border==None:
        neighbor_list_x = self.get_neighbor_list()[1:]
    else:
        neighbor_list_x= neighbor_list_upon_border[1:] # Use the saved list to verfiy the border still same 

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

def start_msg_one_direction(self):
    with self.candidate_to_send_lock:
        if self.id not in self.candidate_to_send:
            self.candidate_to_send.append(self.id)
            self.rec_candidate.append(self.id)

def reset_border_variables(self): 
    self.current_target_ids=[]
    self.candidate_to_send=[]
    self.rec_candidate=[]
    self.border_candidate=False



def Forme_border(self):

    while (self.get_current_spot()['drones_in']>1) or (not(all(neighbor['drones_in'] in [0, 1] for neighbor in self.get_neighbor_list()))) or not self.all_neighbor_spots_owned() and (not self.Emergency_stop.is_set()):
        ''' 
        Before initiating the border procedure, it's important to wait for some time to ensures that the drone is alone in its spot.
        Also wait until all neighbor contains one drone (owner) or empty, in case many are in neighbor spot that would cause change in distrbution becaue 
        of movemnt and the one that moves can be part of the new border so it is better to wait 
        This step eliminates the possibility of erroneously considering a drone as a border-candidate when another drone in the same spot is about to move.
        '''
        '''
        The loop continues as long as any of these conditions are true:

        - The current spot has more than one drone: so it is not alone and one of the drone will populate one of the neigbor spot so ait for that to consider border cndidiate 
        - Any neighbor has more than one drone: so the neighbor has many and some will move and occupy a spot around wait this before check the border  
        - If the spots not owned then should wait, and that is important because while movement a drone can send its destination which can be spot not owned 
            so there is need until the drone arrive and collect the ownership of the spot. 
        '''
        time.sleep(sync_time)
        self.demand_neighbors_info() # return after gathering all info
    
    wait_message_rec = threading.Thread(target=send_msg_border_until_confirmation, args=(self,Forming_border_header)) #pass the function reference and arguments separately to the Thread constructor.
    wait_message_rec.start()
    
    number_of_try=0
    #Continue checking in case of not forming border the process will start again 
    while (not self.Forming_Border_Broadcast_REC.is_set()) and number_of_try<=3 and (not self.expansion_stop.is_set()) and (not self.Emergency_stop.is_set()):
        self.demand_neighbors_info()
        check_border_candidate_eligibility(self)
        if self.border_candidate :
            self.current_target_ids= choose_spot_right_handed(self) # chose spot only when it is candidate 
            self.update_candidate_spot_info_to_neighbors() # Useful if the drone arrived and filled a spot made others sourounded
            self.message_sent_for_border= self.current_target_ids
            '''launch a message circulation for current candidat'''
            start_msg_one_direction(self)
                    
        # This timer will be reset upon each border message is recived 
        # It will be also stopped when forming border broadcast is received 
        # Note in case the border is not formed with absance of new messages, when the timer is up the while loop will re-executed 
        reset_timer_forme_border(self,Forming_border_header)

        while (not self.Emergency_stop.is_set()):
            with self.lock_boder_timer:
                self.remaining_time_forme_border -= 0.5
                if self.remaining_time_forme_border <= 0:
                        break
            time.sleep(0.5)
        
        if self.border_formed== False: 
            number_of_try=number_of_try+1


    if self.border_formed == True:
        self.Forming_Border_Broadcast_REC.wait()
        wait_message_rec.join() # wait wait_message_rec thread to finish and detect the Forming_Border_Broadcast_REC flag
        self.Forming_Border_Broadcast_REC.clear()

        self.demand_neighbors_info() # Update neighbor_list to see the changes in the drones states ( like owner became border)
        self.neighbor_list_upon_border_formation=copy.deepcopy( self.get_neighbor_list()) # Save the Topology arround so it can be used to verfiy the border
        
    reset_border_variables(self)


def confirm_border_connectivity(self):
    if self.get_state()== Border or self.get_state()==Irremovable_boarder: 
        self.current_target_ids= choose_spot_right_handed(self,self.neighbor_list_upon_border_formation) 
        if self.current_target_ids is not None:
            msg= build_border_message(self,Verify_border_header,self.current_target_ids, self.id)
            send_msg(msg)
    
        reset_timer_forme_border(self, Verify_border_header)
        while True:
            with self.lock_boder_timer:
                self.remaining_time_forme_border -= 0.5
                if self.remaining_time_forme_border <= 0:
                        break
            time.sleep(0.5)

    if self.border_verified.is_set():
        print("Border confirmed")
    else:
        print("Border Non confirmed")
        reset_border_variables(self)   
        self.border_verified.clear()


def re_form_border(self):
    if not self.border_verified.is_set():
        if self.get_state() == Border:
            self.change_state_to(Owner)
        if self.get_state() == Irremovable_boarder:
            self.change_state_to(Irremovable)
        Forme_border(self)