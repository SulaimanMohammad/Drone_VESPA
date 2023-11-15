# from .expansion import check_border_candidate_eligibility, send_msg_border_upon_confirmation
import struct
import time 
from .VESPA_module import *
Expan_header= "E"
Arrival_header= "A"
Inherit_header= "I"
Forming_border_header= "F"
Balance_header= "B"

Owner=0
Free=1
Border=2
Irremovable= 3
Irremovable_boarder=4

'''
---------------------------------- Border Communication ------------------------------------
'''
def create_target_list(self, header):
        target_ids=[]
        if header==Balance_header: # Targets are only the border ones
            for s in self.neighbor_list:
                if 'border' in s['states']:
                    border_indices = [idx for idx, state in enumerate(s['states']) if state == 'border']
                    target_ids.extend([s['drones_in_id'][idx] for idx in border_indices])
        else:
            for s in self.neighbor_list:
                target_ids.extend(s["drones_in_id"]) # add the id of all the niegbors including the current
        return target_ids
    
def build_border_message(header, id ,propagation_indicator ,target_ids, candidate):

        # Determine max byte count for numbers
        max_byte_count = max(
                            [determine_max_byte_size(num) for num in target_ids ]+
                            [determine_max_byte_size(num) for num in propagation_indicator]+
                            [determine_max_byte_size(candidate)]
                            )

        # Start message with 'F', followed by max byte count and then the length of the propagation_indicator
        message = header.encode() + struct.pack('>BB', max_byte_count, len(propagation_indicator))

        # Add each number to the message using determined byte count
        for num in propagation_indicator:
            message += num.to_bytes(max_byte_count, 'big')

        message += struct.pack('>B',len(target_ids))
        for num in target_ids:
                message += num.to_bytes(max_byte_count, 'big')
        # Append the sender using the determined byte count
        message += id.to_bytes(max_byte_count, 'big')
        # Append the candidate using the determined byte count
        message += candidate.to_bytes(max_byte_count, 'big')
        message += b'\n'
        return message

def decode_border_message(message):
    # Extract max byte count and lengths of the lists
    max_byte_count, propagation_length = struct.unpack('>BB', message[1:3])
    # Determine the start and end indices for the propagation_indicator numbers
    indices = [(i * max_byte_count + 3, (i + 1) * max_byte_count + 3) for i in range(propagation_length)]
    # Extract the numbers in propagation_indicator
    propagation_indicator = [int.from_bytes(message[start:end], 'big') for start, end in indices]
    # Extract the length of target_ids and then extract the numbers in target_ids
    target_ids_length_position = 3 + max_byte_count * propagation_length
    target_ids_length = struct.unpack('>B', message[target_ids_length_position:target_ids_length_position+1])[0]
    # Determine the start and end indices for the target_ids numbers
    target_ids_start = target_ids_length_position + 1
    indices_target = [(i * max_byte_count + target_ids_start, (i + 1) * max_byte_count + target_ids_start) for i in range(target_ids_length)]
    # Extract the numbers in target_ids
    target_ids = [int.from_bytes(message[start:end], 'big') for start, end in indices_target]
    # Extract the sender
    sender_start = target_ids_start + max_byte_count * target_ids_length
    sender = int.from_bytes(message[sender_start:sender_start+max_byte_count], 'big')
    # Extract the candidate
    candidate_start = sender_start + max_byte_count
    candidate = int.from_bytes(message[candidate_start:candidate_start+max_byte_count], 'big')

    return propagation_indicator, target_ids,sender, candidate

def calculate_propagation_indicator_target(rec_propagation_indicator, all_neighbor):
    # Calculate the target
    # the new target is found based on rec_propagation_indicator so for that we need to find the new one so the reciver can calculate its next target
    # This retuen the unique values in all_neigbors_id that are not present in recieved rec_propagation_indicator
    # rec_propagation_indicator = [0,1, 2, 3] , all_neigbors_id = [0,1,2,4,9] => [4,9]
    new_target_ids= [item for item in rec_propagation_indicator if item not in all_neighbor]

    # Calculated propagation_indicator
    ''' (A ∩ B) ∪ (B - A) = B'''
    # where A is received  propagation_indicator and B is all_neighbor of current drone
    new_propagation_indicator= all_neighbor
    return new_target_ids, new_propagation_indicator

def forward_border_message(self, header, propagation_indicator, targets_ids, candidate):
    '''
    The drone that receives the message will check target_ids, if it is included then it will forward the message
    The message is forwarded to the neigbor drones that have ids are not in the recieved targets_ids and in the propagation_indicator
    Propagation_indicator will help to prevent the messages to have targets from behind
    If the drone is not in the targets then it will ignore the message
    '''
    if self.id in targets_ids: # the current drone is in the targeted ids

        targets_ids= create_target_list(self, header)

        new_propagation_indicator, new_targets_ids= calculate_propagation_indicator_target( propagation_indicator,targets_ids)
        msg= build_border_message(header, self.id,  new_propagation_indicator,new_targets_ids, candidate) # as you see the candidate is resent as it was recived
        send_msg_border_upon_confirmation(self, msg)
    else: # it is not in the tagreted ids then do nothing ( drop the message)
        return

def forward_broadcast_message(self,header,candidate):
    '''
    Build and send messgaes to all the niegbors and since only it is a broadcast
    then propagation_indicator, targets_ids both are [-1].
    The most important is the candidate that fired the broadcast, which is already recieved from neigbors
    Note: since the message will be sent to all the drone around , but rememeber the ones that already received
    it will not recieved it again and th reason is the flag that end the listener is raised and no reading of buffer will be performed
    '''
    msg= build_border_message( header,self.id, [-1] ,[-1],candidate) # as you see the candidate is resent as it was recived
    send_msg(msg)
            
def circle_completed(self):
        if check_border_candidate_eligibility(self):
            self.change_state_to(Border)
            Broadcast_Msg= build_border_message(Forming_border_header,self.id, [-1] ,[-1], self.id)
            send_msg(Broadcast_Msg)
            self.Forming_Border_Broadcast_REC.set() # to end the the loop
        else:
            # the drone got new neigbors and became Free
            if self.state != Free:
                self.change_state_to(Free)

def border_broadcast_respond(self, candidate):
    if candidate in self.rec_candidate: # the sender of broadcast already sent msg to the current drone so it is part of the circle
        # re-check the the droen around still have same situation and still can be border
        if check_border_candidate_eligibility(self):
            self.change_state_to(Border)
    else: # if drone doesnt have the candidate or the sourounding has changed
        self.change_state_to(Free)

def find_msg_direction_forward(self,rec_propagation_indicator,target_ids,sender,candidate ):
    # Only the drone that is candidate will forward messages , the Free or Owner no
    if self.border_candidate == True:
        if sender not in self.rec_propagation_indicator:
            if candidate not in self.rec_candidate:
                self.rec_candidate.append(candidate) # add the received id to the list so when a Broadcast from the same id is recicved that means a full circle include the current drone is completed
            self.rec_propagation_indicator= rec_propagation_indicator # change the propagation_indicator means message from opposite direction has arrived
            self.current_target_ids= forward_border_message(self, Forming_border_header, rec_propagation_indicator, target_ids, candidate)

def build_inherit_message(self,id_rec):
    # Start message with 'I'
    message = Inherit_header.encode()
    max_byte_count = max([determine_max_byte_size(num) for num in self.rec_candidate]+
                            [determine_max_byte_size(id_rec)]+
                            [determine_max_byte_size(num) for num in self.rec_propagation_indicator]
                            )
    #Add the length of rec_candidate list and size of representation of eeach element
    message += struct.pack('>BB', max_byte_count, len(self.rec_candidate))
    # Add the element of the list with a max_byte_count byte representation
    for num in self.rec_candidate:
        message += num.to_bytes(max_byte_count, 'big')
    #Add the length of rec_propagation_indicator list
    message += struct.pack('>B', len(self.rec_propagation_indicator))
    # Add the element of rec_propagation_indicator list with a max_byte_count byte representation
    for num in self.rec_propagation_indicator:
        message += num.to_bytes(max_byte_count, 'big')
    message +=id_rec.to_bytes(max_byte_count, 'big')
    message += b'\n'
    return message

def decode_inherit_message(message):
    index = len(Inherit_header.encode())
    # Extract max_byte_count and the length of rec_candidate list
    max_byte_count, rec_candidate_length = struct.unpack('>BB', message[index:index+2])
    index += 2
    # Decode the numbers in rec_candidate
    rec_candidate_values = []
    for _ in range(rec_candidate_length):
        num = int.from_bytes(message[index:index+max_byte_count], 'big')
        rec_candidate_values.append(num)
        index += max_byte_count
    # Extract the length of rec_propagation_indicator list
    rec_propagation_indicator_length = struct.unpack('>B', message[index:index+1])[0]
    index += 1
    # Decode the numbers in rec_propagation_indicator
    rec_propagation_indicator_values = []
    for _ in range(rec_propagation_indicator_length):
        num = int.from_bytes(message[index:index+max_byte_count], 'big')
        rec_propagation_indicator_values.append(num)
        index += max_byte_count
    # Decode id_rec
    id_rec = int.from_bytes(message[index:index+max_byte_count], 'big')
    return rec_candidate_values, rec_propagation_indicator_values, id_rec

def handel_broken_into_spot(self, msg):
    if self.border_candidate== True:
        positionX, positionY, state, previous_state,id_rec= self.decode_spot_info_message(msg)
        self.update_neighbors_list(positionX, positionY, state, previous_state,id_rec) # No need to mutex since the drone is in border_candidate only in it was Owner and reserved spot
        check_border_candidate_eligibility(self) # use only the updated list and see if the current drone still candidate
        if self.border_candidate == False: # changed due to 6 neigbors filled
            self.change_state_to(Free) # Has 6 neighbors
            if not self.rec_candidate: # if the rec_candidate is not empty, means messages for border are already received
                msg= build_inherit_message(self, id_rec) # id_rec is the id of the drone hopped in
                send_msg(msg)

def handel_inheritence_message(self, msg):
        new_rec_candidate_values, new_rec_propagation_indicator_values, id_rec= decode_inherit_message(msg)
        if id_rec== self.id:
            self.update_rec_candidate(new_rec_candidate_values)
            self.update_rec_propagation_indicator(new_rec_propagation_indicator_values)
