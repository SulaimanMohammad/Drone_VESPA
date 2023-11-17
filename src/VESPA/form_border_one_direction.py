from .VESPA_module import *
# from .expansion import check_border_candidate_eligibility , send_msg_border_upon_confirmation
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

set_env(globals())
'''
This module is used to send messages by sending message only to one drone by searching clockwise from empty spot 
'''
def choose_spot_right_handed(self):
    # Start from index 1 to skip 's0'
    neighbors_without_s0 = self.neighbor_list[1:]

    # Variable to keep track of the last drones_in_id
    last_drones_in_id = None
    for neighbor in neighbors_without_s0:
        if neighbor["drones_in"] > 0:
            last_drones_in_id = neighbor["drones_in_id"]
        elif neighbor["drones_in"] == 0 and last_drones_in_id is not None:
            # Found the first spot with drones_in == 0 after finding a spot with drones_in > 0
            return last_drones_in_id

def build_border_message_right_handed(self,header,target_ids, candidate):

    # Determine max byte count for numbers
    max_byte_count = max(
                        [self.determine_max_byte_size(num) for num in target_ids ]+
                        [self.determine_max_byte_size(candidate)]+
                        [self.determine_max_byte_size(id)]
                        )
    # Start message with 'F', followed by max byte count and then the length of the propagation_indicator
    message = header.encode() + struct.pack('>BB', max_byte_count, len(target_ids))

    message += struct.pack('>B',len(target_ids))
    for num in target_ids:
            message += num.to_bytes(max_byte_count, 'big')
    # Append the sender using the determined byte count
    message += self.id.to_bytes(max_byte_count, 'big')
    # Append the candidate using the determined byte count
    message += candidate.to_bytes(max_byte_count, 'big')
    message += b'\n'
    return message

def decode_border_message_right_handed(message):
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
        target_id = int.from_bytes(num_bytes, 'big')
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

    # Assuming there's a newline at the end
    if message.startswith(b'\n'):
        message = message[1:]

    return sender_id, target_ids, candidate

def circle_completed(self):
        if check_border_candidate_eligibility(self):
            self.change_state_to(Border)
            Broadcast_Msg= build_border_message_right_handed(self,Forming_border_header,[-1], self.id)
            send_msg_border_upon_confirmation(self, Broadcast_Msg)
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


def forward_broadcast_message(self,header,candidate):
    '''
    Build and send messgaes to all the niegbors and since only it is a broadcast
    then propagation_indicator, targets_ids both are [-1].
    The most important is the candidate that fired the broadcast, which is already recieved from neigbors
    Note: since the message will be sent to all the drone around , but rememeber the ones that already received
    it will not recieved it again and th reason is the flag that end the listener is raised and no reading of buffer will be performed
    '''
    msg= build_border_message_right_handed(self, header,[-1],candidate) # as you see the candidate is resent as it was recived
    send_msg(msg)

def border_broadcast_respond(self, candidate):
    if candidate in self.rec_candidate: # the sender of broadcast already sent msg to the current drone so it is part of the circle
        # re-check the the droen around still have same situation and still can be border
        if check_border_candidate_eligibility(self):
            self.change_state_to(Border)
    else: # if drone doesnt have the candidate or the sourounding has changed
        self.change_state_to(Free)

def form_border_one_direction(self,header,msg):
    sender_id, target_ids, candidate= decode_border_message_right_handed(msg)
    if len(target_ids)==1 and target_ids[0]==-1:
                if self.border_candidate==True:
                    border_broadcast_respond(self, candidate)
                # Here any drone in any state needs to forward the boradcast message and rise ending flag
                forward_broadcast_message(self, Forming_border_header,candidate)
                self.Forming_Border_Broadcast_REC.set()

    if self.id in  target_ids:
        if self.id == candidate:
            circle_completed()
        else: 
            if self.border_candidate == True:
                if candidate not in self.rec_candidate:
                    self.rec_candidate.append(candidate)
            build_border_message_right_handed(self,header,target_ids, candidate)
            self.current_target_ids= choose_spot_right_handed(self)
            msg= build_border_message_right_handed(self,header,self.current_target_ids, candidate)
            send_msg_border_upon_confirmation(self, msg)

def start_msg_one_direction(self,header):
    self.current_target_ids= choose_spot_right_handed()
    msg= build_border_message_right_handed(self,header,self.current_target_ids, self.id)
    send_msg_border_upon_confirmation(self, msg)