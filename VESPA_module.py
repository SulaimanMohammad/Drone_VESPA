from enum import Enum
from math import sqrt
import random
import copy
import struct
from drone_ardupilot import *

'''
-------------------------------------------------------------------------------------
---------------------------------- Variables ----------------------------------------
-------------------------------------------------------------------------------------
'''

sq3=sqrt(3)
a= 20 # drone range
'''aim for a buffer zone. This ensures that even if there's a slight miscalculation in the range or
a sudden change in environmental conditions, there's some leeway before communication is lost.'''
a=a*0.95
C=100
eps=20
speed_of_drone=2 # 2 m/s
movement_time= a*speed_of_drone*(1+ 0.2)  # add 20% of time for safty
scanning_time= 10 # in second ( TODO function to the speed and size of a )
sync_time= 10
multiplier = 100
defined_groundspeed=1

DIR_VECTORS = [
    [0, 0],    # s0 // don't move, stay
    [90, a],   # s1
    [30, a],   # s2
    [330, a],  # s3
    [270, a],  # s4
    [210, a],  # s5
    [150, a]   # s6
]

DIR_xy_distance_VECTORS = [
    [0, 0],                              # s0 // don't move, stay
    [(sq3 * a), 0],                      # s1
    [(sq3 / 2.0) * a, (3.0 / 2.0) * a],  # s2
    [-(sq3 / 2) * a, (3.0 / 2.0) * a],   # s3
    [-sq3 * a, 0],                       # s4
    [-(sq3 / 2.0) * a, -(3.0/ 2.0) * a], # s5
    [(sq3 / 2.0) * a, -(3.0 / 2.0) * a]  # s6
]

def update_DIR_xy_distance_VECTORS():
    global DIR_xy_distance_VECTORS
    DIR_xy_distance_VECTORS = [
    [0, 0],                               # s0 // don't move, stay
    [(sq3 * a), 0],                       # s1
    [(sq3 / 2.0) * a, (3.0 / 2.0) * a],   # s2
    [-(sq3 / 2.0) * a, (3.0 / 2.0) * a],  # s3
    [-sq3 * a, 0],                        # s4
    [-(sq3 / 2.0) * a, -(3.0/ 2.0) * a],  # s5
    [(sq3 / 2.0) * a, -(3.0 / 2.0) * a]   # s6
    ]

def set_a(val):
    global a
    a= val
    update_DIR_xy_distance_VECTORS() # changing of a should change the direction distances


formula_dict = {
    "s0": "sqrt(DxDy2)",
    "s1": "sqrt(DxDy3a2 + a * aDx)",
    "s2": "sqrt(DxDy3a2 + a * (sqDx + (3 * Dy)))",
    "s3": "sqrt(DxDy3a2 + a * (3 * Dy - sqDx))",
    "s4": "sqrt(DxDy3a2 - (aDx*a))",
    "s5": "sqrt(DxDy3a2 - a * (sqDx + (3 * Dy)))",
    "s6": "sqrt(DxDy3a2 - a * (3 * Dy - sqDx))"
}


Owner=0
Free=1
Border=2
Irremovable= 3
Irremovable_boarder=4

'''
These will be used in the building and decoding message
'''
#Exchanging data Headers
Movement_command= "M"
Calibration= "C"
Demand_header= "D"
Reponse_header= "R"
# Expansion Headers
Expan_header= "E"
Arrival_header= "A"
Inherit_header= "I"
Forming_border_header= "F"
#Spanning Headers
Spanning_header= "S"
# Blancing Headers
Local_balance_header="L"
Guidance_header= "G"
Balance_header= "B"



'''
-------------------------------------------------------------------------------------
----------------------------- Calss members, init__ ---------------------------------
-------------------------------------------------------------------------------------
'''
class Drone:
    def __init__(self, vehicle, x,y,z):
        self.positionX=x
        self.positionY=y
        self.distance_from_sink=0 # the distance of the drone from  the sink
        self.hight=z
        self.state=1
        self.previous_state=1
        self.drone_id_to_sink=0
        self.drone_id_to_border=0
        self.a=a
        self.border_candidate=False
        self.dominated_direction=0
        self.phase= Expan_header
        self.spots_to_check_for_border=[]
        self.rec_candidate=[] # contains ids of drone that fired a messaging circle
        self.drone_id_to_sink=[]
        self.drone_id_to_border=[]
        self. min_distance_dicts=[] # nigboor close to the sink
        self.border_neighbors=[] # contains the spots that are occupied while forming the border
        self.allowed_spots=[]
        self.direction_taken=[]  # direction path (spots) that are taken in the phase
        self.neighbor_list = []  # list that contains the 6 neighbors around the current location
        self.rec_propagation_indicator=[]
        self.id=self.get_id()
        set_a(self.read_xbee_range()) # read last value of a and set it in the code  
        # init s0 and it will be part of the spots list
        self.neighbor_list=[{"name": "s" + str(0), "distance": 0, "priority": 0, "drones_in": 1,"drones_in_id":[], "states": [] , "previous_state": []}]
        # save the first spot which is s0 the current place of the drone
        # spot is another name of s_list[0] so any changes will be seen in spot
        self.spot= self.neighbor_list[0]
        self.spot["id"]=self.id
        self.spot["states"]=self.state
        self.spot["previous_state"]=self.state
        self.num_neigbors = 6
        for i in range(1, self.num_neigbors+1):
            s = {"name": "s" + str(i), "distance": 0, "priority": 0,"drones_in": 0,"drones_in_id":[] , "states": [], "previous_state": []}
            self.neighbor_list.append(s)
        self.lock_state = threading.Lock()
        self.lock_neighbor_list = threading.Lock()
        self.neighbors_list_updated = threading.Event()
        self.start_expanding= threading.Event()
        # Lance a thread to read messages continuously
        self.xbee_receive_message_thread = threading.Thread(target=self.receive_message, args=(self,vehicle)) #pass the function reference and arguments separately to the Thread constructor.
        self.xbee_receive_message_thread.start()

    '''
    -------------------------------------------------------------------------------------
    ---------------------------------- Communication ------------------------------------
    -------------------------------------------------------------------------------------
    '''
    def build_data_demand_message(self):
        return Demand_header.encode() + b'\n'

    def demand_neighbors_info(self):
        demand_msg= self.build_data_demand_message()
        self.send_msg(demand_msg)
        self.neighbors_list_updated.wait()
        self.neighbors_list_updated.clear()

    def get_neighbors_info(self):
        '''
        In case The demand functoin is done becuse a message with different headr between the respons msg
        then the thi function will set again the flag
        but since the demand doesnt exit then need to be sure it is always cleared befoe use'''
        if self.neighbors_list_updated.is_set():
            self.neighbors_list_updated.clear()
        time.sleep(1) # Wait to have all msgs
        # Retrive all the reponse messages then rise the flage that all is received
        while msg.startswith(Reponse_header.encode()):
            positionX, positionY, state, previous_state,id_value= self.decode_spot_info_message(msg)
            self.update_neighbors_list(positionX, positionY, state, previous_state,id_value)
            msg= self.retrive_msg_from_buffer()
        self.neighbors_list_updated.set()

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

    #Return the byte count necessary to represent max ID.
    def determine_max_byte_size (slef,number):
        if number <= 0xFF:  # 1 byte
            return 1
        elif number <= 0xFFFF:  # 2 bytes
            return 2
        elif number <= 0xFFFFFF:  # 3 bytes
            return 3
        else:
            return 4  # max 4 bytes

    def build_border_message(self, header, propagation_indicator ,target_ids, candidate):

        # Determine max byte count for numbers
        max_byte_count = max(
                            [self.determine_max_byte_size(num) for num in target_ids ]+
                            [self.determine_max_byte_size(num) for num in propagation_indicator]+
                            [self.determine_max_byte_size(candidate)]
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
        message += self.id.to_bytes(max_byte_count, 'big')
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

    def calculate_propagation_indicator_target(self,rec_propagation_indicator, all_neighbor):
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

            targets_ids= self.creat_target_list(header)

            new_propagation_indicator, new_targets_ids= self.calculate_propagation_indicator_target( propagation_indicator,targets_ids)
            msg= self.build_border_message(header, new_propagation_indicator,new_targets_ids, candidate) # as you see the candidate is resent as it was recived
            self.send_msg(msg)
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
        msg= self.build_border_message(header, [-1] ,[-1],candidate) # as you see the candidate is resent as it was recived
        self.send_msg(msg)

    def send_msg(self,msg):
        #TODO deal with sending boradcasting
        pass

    def retrive_msg_from_buffer(self):
        # device is the object of Xbee connection
        xbee_message = device.read_data(0.02) #  0.02 timeout in seconds
        # read untile the bufere is empty or retrive 7 msgs
        msg=" "
        if xbee_message is not None:
            return msg
        else:
            return None

    def encode_float_to_int(self, value, multiplier=100):
        """Encodes a float as an integer with a given multiplier."""
        encoded = int(value * multiplier)
        # Choose the format based on the size of the integer
        format_char = '>H' if encoded <= 65535 else '>I'
        return struct.pack(format_char, encoded)

    def decode_int_to_float(self, encoded_float):
        # Check the byte length to decide the format
        if len(encoded_float) == 2:
            value = struct.unpack('>H', encoded_float)[0]
        elif len(encoded_float) == 4:
            value = struct.unpack('>I', encoded_float)[0]
        # Convert back to float
        return value / multiplier  # Assuming multiplier = 100

    def build_spot_info_message(self, header):
        # Start message with 'E'
        message = header.encode()
        # Encode positionX and positionY
        message += self.encode_float_to_int(self.positionX)
        message += self.encode_float_to_int(self.positionY)
        # Determine and append max byte count for self.id
        max_byte_count = self.determine_max_byte_size(self.id)
        message += struct.pack('>B', max_byte_count)  # Note the single B format
        # Encode state
        message += struct.pack('>B', self.state)
        message += struct.pack('>B', self.previous_state)
        # Append the id itself
        message += self.id.to_bytes(max_byte_count, 'big')
        message += b'\n'
        return message

    def decode_spot_info_message(self,message):
        # Starting index after the header
        index = 1 #All headers are 1 byte
        # Decode positionX
        positionX_encoded = message[index:index+2] if len(message) - index == 6 else message[index:index+4]
        positionX = self.decode_int_to_float(positionX_encoded)
        index += len(positionX_encoded)
        # Decode positionY
        positionY_encoded = message[index:index+2] if len(message) - index == 6 else message[index:index+4]
        positionY = self.decode_int_to_float(positionY_encoded)
        index += len(positionY_encoded)
         # Decode state and previous_state
        state = struct.unpack('>B', message[index:index+1])[0]
        index += 1
        previous_state = struct.unpack('>B', message[index:index+1])[0]
        index += 1
        # Decode max_byte_count for id
        max_byte_count = struct.unpack('>B', message[index:index+1])[0]
        index += 1
        # Decode self.id
        id_value = int.from_bytes(message[index:index+max_byte_count], 'big')
        return positionX, positionY, state, previous_state, id_value
  
    def circle_completed(self):
        if self.check_border_candidate_eligibility():
            self.change_state_to(Border)
            Broadcast_Msg= self.build_border_message(Forming_border_header,[-1] ,[-1], self.id)
            self.send_msg(Broadcast_Msg)
            self.Forming_Border_Broadcast_REC.set() # to end the the loop
        else:
            # the drone got new neigbors and became Free
            if self.state != Free:
                self.change_state_to(Free)

    def border_broadcast_respond(self, candidate):
        if candidate in self.rec_candidate: # the sender of broadcast already sent msg to the current drone so it is part of the circle
            # re-check the the droen around still have same situation and still can be border
            if self.check_border_candidate_eligibility():
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
                self.forward_border_message(Forming_border_header, rec_propagation_indicator, target_ids, candidate)

    def handel_broken_into_spot(self, msg):
        if self.border_candidate== True:
            positionX, positionY, state, previous_state,id_rec= self.decode_spot_info_message(msg)
            self.update_neighbors_list(positionX, positionY, state, id_rec) # No need to mutex since the drone is in border_candidate only in it was Owner and reserved spot
            self.check_border_candidate_eligibility(observe=False) # use only the upddated list and see if the current drone still candidate
            if self.border_candidate == False: # changed due to 6 neigbors filled
                self.change_state_to(Free) # Has 6 neighbors
                if not self.rec_candidate: # if the rec_candidate is not empty, means messages for border are already received
                    msg=self.build_inherit_message(id_rec) # id_rec is the id of the drone hopped in
                    self.send_msg(msg)

    def clear_buffer(self):
        # read the buffer until it is empty
        # while xbee_device.get_queue_length() > 0:
        #     xbee_device.read_data()
        pass


    '''
    -------------------------------------------------------------------------------------
    -------------------------------- Update upon movement--------------------------------
    -------------------------------------------------------------------------------------
    '''
    def get_id(self):
        with open('Operational_Data.txt', 'r') as file:
            for line in file:
                # Check if line contains max_acceleration
                if "id" in line:
                    id = int(line.split('=')[1].strip())
                return id

    def read_xbee_range(self):
        with open('Operational_Data.txt', 'r') as file:
            for line in file:
                # Check if line contains max_acceleration
                if "a" in line:
                    xbee_range = int(line.split('=')[1].strip())
                return xbee_range
    
    def update_xbee_range(self,new_a):
        with open('Operational_Data.txt', 'w') as file:
            file.write(f'a = {new_a}\n')
 
    def get_state(self):
        with self.lock_state:
            return self.state

    def write_state(self, state):
        with self.lock_state:
            self.state= state

    def change_state_to( self, new_state):
            with self.lock_state:
                self.previous_state= self.state # save the preivious state
                self.spot["previous_state"][0]= self.state
                self.state= new_state # change the state
                self.spot["states"][0]= self.state

    def check_Ownership(self):
        self.check_num_drones_in_neigbors()
         #if s0 where the drone is conatins only the drone ( droen Owner)
         # the done as you see count itself at the spot
         # the drone should be Owner to be free
        print("drone in s0", self.spot["drones_in"] )
        if self.spot["drones_in"]==1: # the drone is Owner
            self.change_state_to (Owner)
        print("drone state in s0", self.state )

    def update_rec_candidate(self, new_rec_candidate):
        # Update without duplicates
        for item in new_rec_candidate:
            if item not in self.rec_candidate:
                self.rec_candidate.append(item)

    def update_rec_propagation_indicator(self, new_rec_propagation_indicator):
        # Update without duplicates
        for item in new_rec_propagation_indicator:
            if item not in self.rec_propagation_indicator:
                self.rec_propagation_indicator.append(item)

    # PositionX , positionY are the coordinates from the sink
    # Format: _.xx 2 decimal
    def update_location(self, dir):
        x=  DIR_xy_distance_VECTORS[dir][0]
        y= DIR_xy_distance_VECTORS[dir][1]
        self.positionX =round(self.positionX + x,2) #add the value not assign because it is movement
        self.positionY = round(self.positionY+ y ,2)
        # Find the distance of the neigboors at the new position
        self.calculate_neighbors_distance_sink()

    def update_candidate_spot_info_to_neighbors(self):
        msg= self.build_spot_info_message(Arrival_header)
        self.send_msg(msg)

    def update_neighbors_list(self, positionX, positionY, state, previous_state, id_rec):
        s_index= self.find_relative_spot(positionX, positionY)
        # Check if index is valid ( index between 0 and 6 )
        if 0 <= s_index <= self.num_neigbors+1:
            with self.lock_neighbor_list:
                # Check if the id_rec of the drone is already in neighbor_list
                for s in self.neighbor_list:
                    if id_rec in s["drones_in_id"]:
                        # The drone was in another neighborhood spot
                        if int(s["name"][1:]) != s_index :
                            # Find the index of the drone_id
                            idx = s['drones_in_id'].index(id_rec)
                            # Remove from drones_in_id and states
                            s['drones_in_id'].pop(idx)
                            s['states'].pop(idx)
                            s['previous_state'].pop(idx)
                            # Decrement drones_in count
                            s['drones_in'] -= 1
                        else: # The id of drone is in the same spot just update the satates
                            s['states']= state
                            s['previous_state']=previous_state
                            return  # Exit the function

                # If the ID of drone doesnt exist at all or it was deleted after was in another neighborhood spot
                # Retrieve the corresponding spot
                s = self.neighbor_list[s_index]
                # Append drone_id to drones_in_id and state_rec to states
                s['drones_in_id'].append(id_rec)
                s['states'].append(state)
                s['previous_state'].append(previous_state)
                # Increment drones_in count
                s['drones_in'] += 1
        else:
            print("Invalid index provided")

    # Change information about the state of one of neighbors
    def update_state_in_neighbors_list( self, id, state):
        # Lock to avoid any data race since there are two threads ( main , xbee)
        with self.lock_neighbor_list:
            for s in self.neighbor_list:
                if id in s["drones_in_id"]:
                    # Find the index of the drone_id and changr the state
                    idx = s['drones_in_id'].index(id)
                    s['states'][idx]= state