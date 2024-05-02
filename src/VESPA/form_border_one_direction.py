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
        #if check_border_candidate_eligibility(self):
        if self.border_candidate:
            self.change_state_to(Border)
            Broadcast_Msg= build_border_message(self,Forming_border_header,[-1], self.id)
            #send_msg_border_upon_confirmation(self, Broadcast_Msg)
            send_msg(Broadcast_Msg) # bordacst doent need to be waiting conformation 
            now = datetime.now()
            print(now.strftime("%H:%M:%S.%f"), "end pahse")

            self.Forming_Border_Broadcast_REC.set() # to end the the loop
        else:
            # the drone got new neigbors and became Free
            if self.state != Free:
                self.change_state_to(Free)

def border_broadcast_respond(self, candidate):
    if candidate in self.rec_candidate: # the sender of broadcast already sent msg to the current drone so it is part of the circle
        # re-check the the droen around still have same situation and still can be border
        #if check_border_candidate_eligibility(self):
        if self.border_candidate:
            self.change_state_to(Border)
    else: # if drone doesnt have the candidate or the sourounding has changed
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
    if not self.Forming_Border_Broadcast_REC.is_set():
        sender_id, target_ids, candidate= decode_border_message(msg)
        reset_timer_forme_border(self,header )
        now = datetime.now()
        print(now.strftime("%H:%M:%S.%f"), ":REC Sender_id, target_ids, candidate, elg",sender_id, target_ids, candidate, self.border_candidate )
        # the sender of message was in the self.current_target_ids means it recived a message and reponde to it  
        #check_border_candidate_eligibility(self)
        #self.current_target_ids= choose_spot_right_handed(self)
        if sender_id in self.current_target_ids and candidate in self.rec_candidate:
            print( "                                MESSAGR REC", sender_id)
            with self.sending_messgae_list:
                if candidate in self.messages_to_be_sent:
                    self.messages_to_be_sent.remove(candidate)

        if len(target_ids)==1 and target_ids[0]==-1:
            if self.border_candidate==True:
                border_broadcast_respond(self, candidate)
            # Here any drone in any state needs to forward the boradcast message and rise ending flag
            forward_broadcast_message(self, Forming_border_header,candidate)
            finish_timer_forme_border(self)
            self.Forming_Border_Broadcast_REC.set()

        if self.id in  target_ids and target_ids :# targets exist not empty s
            if self.id == candidate:
                print("sender, message_sent_for_border", sender_id, self.message_sent_for_border)
                if sender_id in self.message_sent_for_border:
                    print ("message backwarrds")
                    self.border_formed=False
                    finish_timer_forme_border(self)

                else: 
                    circle_completed(self)
                    self.border_formed= True
                    finish_timer_forme_border(self)

            else: 
                #print(" from listener")
            
                    # self.current_target_ids= choose_spot_right_handed(self)
                    # msg= build_border_message(self,header,self.current_target_ids, candidate)
                    # print(" forward messae to",self.current_target_ids )
                with self.sending_messgae_list:
                    if candidate not in self.messages_to_be_sent:
                        self.messages_to_be_sent.append(candidate)
                        #send_waiting_receiving(self, header, candidate)
                    #send_msg(msg)
                
def send_msg_border_until_confirmation(self):
    while not self.Forming_Border_Broadcast_REC.is_set() and  (not self.expansion_stop.is_set()) and (not self.Emergency_stop.is_set()):
        header= Forming_border_header
        # that is  needed or it will be blocked trying to send same candidate 
        candidates_to_process = []

        with self.sending_messgae_list:
            if  self.messages_to_be_sent :# it is empty 
                #candidate= self.messages_to_be_sent.pop(0)
                candidates_to_process = list(self.messages_to_be_sent)
        
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
                msg=revaluate_reforward(self,header, candidate)
                now = datetime.now()
                print(now.strftime("%H:%M:%S.%f"), ": SENDSender_id, target_ids, candidate",self.id, self.current_target_ids, candidate )

                send_msg(msg)
                time.sleep(exchange_data_latency)# time untile the message arrives 
        time.sleep(exchange_data_latency)


def verify_border(self,header, msg):
    if not self.border_verified.is_set():
        sender_id, target_ids, candidate= decode_border_message(msg)
        reset_timer_forme_border(self, Verify_Border)

        now = datetime.now()
        print(now.strftime("%H:%M:%S.%f"), ":VERY Sender_id, target_ids, candidate, elg",sender_id, target_ids, candidate, self.border_candidate )

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
                    self.current_target_ids= choose_spot_right_handed(self)
                    msg= build_border_message(self,header,self.current_target_ids, candidate)
                    send_msg(msg) 


''''*
-------------------------------------------------------------------------------------
----------------------------------- Main functions ----------------------------------
-------------------------------------------------------------------------------------
'''
def reset_timer_forme_border(self, header):
    with self.lock_boder_timer:
        if header== Forming_border_header:
            self.remaining_time_forme_border=10 # contains waiting and confim 
        else:
            self.remaining_time_forme_border=5

# called by other threads 
def finish_timer_forme_border(self):
    with self.lock_boder_timer:
        self.remaining_time_forme_border=0


def revaluate_reforward(self,header, candidate):
    if self.current_target_ids is not None:
        msg= build_border_message(self,header,self.current_target_ids, candidate)   
    return msg 


    
def check_border_candidate_eligibility(self):
    # self.list_finished_update.wait()
    if self.get_state() != Owner: 
        self.border_candidate=False
        return self.border_candidate
    # self.demand_neighbors_info() # return after gathering all info
    #self.correct_states_after_comm()
    # print("from chekc border")
    # for station in self.neighbor_list:
    #     if station['drones_in'] > 0:
    #         print(station)

    self.border_candidate=False
    # self.dominated_direction= count_element_occurrences(self)
    # # Define a dictionary to map directions to the corresponding spots_to_check_for_border values
    # direction_to_check_map = {
    #     0: [1,2,3,4,5,6],
    #     1: [1, 2, 6],
    #     2: [1, 2, 3],
    #     3: [3, 2, 4],
    #     4: [3, 4, 5],
    #     5: [4, 5, 6],
    #     6: [1, 5, 6]
    # }
    # self.spots_to_check_for_border=direction_to_check_map.get(self.dominated_direction, [])
    unoccupied_spots_counter = 0
    for neighbor in self.get_neighbor_list():
        # Find the corresponding entry in neighbor_list by its name
        #neighbor = next((n for n in copied_neigborlist if n["name"] == "s" + str(check)), None)
        if neighbor["drones_in"] == 0: # spot also is not occupied
            unoccupied_spots_counter += 1

    if unoccupied_spots_counter>0 and self.spot ["drones_in"]==1 and self.state==Owner: # at least one spot is empty so the drone can be part of he border
        print("all_neighbor_spots_owned", self.all_neighbor_spots_owned() )
        if  self.all_neighbor_spots_owned(): 
            self.border_candidate=True
    else: 
        self.border_candidate=False
        
    return self.border_candidate

def choose_spot_right_handed(self, neighbor_list_upon_border=None):
    # self.list_finished_update.wait()
    if neighbor_list_upon_border==None:
        neighbor_list_x = self.get_neighbor_list()[1:]
    else:
        print("use the saved list")
        neighbor_list_x= neighbor_list_upon_border[1:]
    
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
    return []

def start_msg_one_direction(self,header):
    # self.current_target_ids= choose_spot_right_handed(self)
    # if self.current_target_ids: # send message only if there is target found ( to avoid send wronf message) 
    #     msg= build_border_message(self,header,self.current_target_ids, self.id)
    print(" send messae to form border " )
    with self.sending_messgae_list:
        if self.id not in self.messages_to_be_sent:
            self.messages_to_be_sent.append(self.id)
            self.rec_candidate.append(self.id)
    self.participated= True
    #send_waiting_receiving(self, header, self.id)
        #send_msg(msg)

def confirm_border_connection(self):
    header= Verify_Border
    if self.get_state()== Border or self.get_state()==Irremovable_boarder: 
        self.current_target_ids= choose_spot_right_handed(self,self.neighbor_list_upon_border_formation) 
        if self.current_target_ids is not None:
            msg= build_border_message(self,header,self.current_target_ids, self.id)
            send_msg(msg)
            print("message sent ", self.id, self.current_target_ids )
    
        reset_timer_forme_border(self, Verify_Border)
        while True:
            with self.lock_boder_timer:
                self.remaining_time_forme_border -= 0.5
                if self.remaining_time_forme_border <= 0:
                        break
            time.sleep(0.5)

    if self.border_verified.is_set():
        print("border confirmed")
    else:
        print("border NOn confirmed")
        self.current_target_ids=[]
        self.messages_to_be_sent=[]
        self.rec_candidate=[]
        self.border_candidate=False
        print("self.state",self.state)
        self.change_state_to(Owner)
        print("self.state",self.state)
        Forme_border(self)
    self.border_verified.clear()



def Forme_border(self):
    wait_message_rec = threading.Thread(target=send_msg_border_until_confirmation, args=(self,)) #pass the function reference and arguments separately to the Thread constructor.
    wait_message_rec.start()
    number_of_try=0
    # need to continue checking because it can be not border but can be after 
    while (not self.Forming_Border_Broadcast_REC.is_set()) and number_of_try<=3 and (not self.expansion_stop.is_set()) and (not self.Emergency_stop.is_set()):
        print(" send demane from Forme_border")
        self.demand_neighbors_info()
        print( "Done reading data")
        for station in self.get_neighbor_list():
            if station['drones_in'] > 0:
                print(station)

        check_border_candidate_eligibility(self)
        print("self.border_candidate",self.border_candidate )

        if self.border_candidate :
            self.current_target_ids= choose_spot_right_handed(self) # chose spot only when it is candidate 
            self.message_sent_for_border= self.current_target_ids
            print(  self.message_sent_for_border)
            print( "current_target_ids", self.current_target_ids)
            self.update_candidate_spot_info_to_neighbors() # Useful if the drone arrived and filled a spot made others sourounded
            print( " it is elgibale ")
            '''launch a message circulation for current candidat'''
            start_msg_one_direction(self,Forming_border_header)
            print("self.messages_to_be_sent", self.messages_to_be_sent)
        
        reset_timer_forme_border(self,Forming_border_header )
        # This will be broke in case the the timer is up or the emergency is done 
        while (not self.Emergency_stop.is_set()):
            with self.lock_boder_timer:
                self.remaining_time_forme_border -= 0.5
                if self.remaining_time_forme_border <= 0:
                        break 
            time.sleep(0.5)
        if self.border_formed== False: 
            number_of_try=number_of_try+1
        
        print("NEW ITERZTOion , border_formed, number_of_try", self.border_formed,number_of_try)
    
    print (" BREAK THE LOOP ")
    if self.border_formed== False:
        print(" need to abort mession")
       
    else: 
        self.Forming_Border_Broadcast_REC.wait()
        wait_message_rec.join() # wait it to be done 
        self.Forming_Border_Broadcast_REC.clear()
        self.demand_neighbors_info() # needed to update what neigbor become border 
        self.neighbor_list_upon_border_formation=copy.deepcopy( self.get_neighbor_list())

