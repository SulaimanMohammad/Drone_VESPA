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
C=100
eps=20
speed_of_drone=2 # 2 m/s 
movement_time= a*speed_of_drone*(1+ 0.2)  # add 20% of time for safty 
scanning_time= 10 # in second ( TODO function to the speed and size of a )
sync_time= 10
multiplier = 100

DIR_VECTORS = [
    [0, 0],                             # s0 // don't move, stay
    [90, a],            # s1
    [30, a],   # s2
    [330, a],  # s3
    [270, a],             # s4
    [210, a], # s5
    [150, a]   # s6
]

DIR_xy_distance_VECTORS = [
    [0, 0],                             # s0 // don't move, stay
    [(sq3 * a), 0],            # s1
    [(sq3 / 2.0) * a, (3.0 / 2.0) * a],   # s2
    [-(sq3 / 2) * a, (3.0 / 2.0) * a],  # s3
    [-sq3 * a, 0],             # s4
    [-(sq3 / 2.0) * a, -(3.0/ 2.0) * a], # s5
    [(sq3 / 2.0) * a, -(3.0 / 2.0) * a]   # s6
]

def update_DIR_xy_distance_VECTORS():
    global DIR_xy_distance_VECTORS
    DIR_xy_distance_VECTORS = [
    [0, 0],                             # s0 // don't move, stay
    [(sq3 * a), 0],            # s1
    [(sq3 / 2.0) * a, (3.0 / 2.0) * a],   # s2
    [-(sq3 / 2.0) * a, (3.0 / 2.0) * a],  # s3
    [-sq3 * a, 0],             # s4
    [-(sq3 / 2.0) * a, -(3.0/ 2.0) * a], # s5
    [(sq3 / 2.0) * a, -(3.0 / 2.0) * a]   # s6
    ]

def set_a( val):
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


'''
phase= E expansion
phase= F forming border
phase= S Spaning  
pahse= B balancing 
This will be used in the message 
'''

Owner=0
Free=1
Border=2
Irremovable= 3 
Irremovable_boarder=4 

Expan_header= "E"
Arrival_header= "A"
Inherit_header= "I" 
Forming_border_header= "F"
Span_header= "S"
Balance_header= "B"


'''
-------------------------------------------------------------------------------------
----------------------------- Calss members, init__ ---------------------------------
-------------------------------------------------------------------------------------
'''
class Drone: 
    def __init__(self, x,y,z):
        self.positionX=x
        self.positionY=y
        self.distance_from_sink=0 # the distance of the drone from  the sink 
        self.id=0
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
        self.drone_id_to_sink
        self.drone_id_to_border
        self. min_distance_dicts=[] # nigboor close to the sink 
        self.border_neighbors=[] # contains the spots that are occupied while forming the border
        self.direction_taken=[]  # direction path (spots) that are taken in the phase 
        self.neighbor_list = []  # list that contains the 6 neighbors around the current location
        self.rec_propagation_indicator=[]
        # init s0 and it will be part of the spots list 
        self.neighbor_list=[{"name": "s" + str(0), "distance": 0, "priority": 0, "drones_in": 1,"drones_in_id":[], "states": [] , "previous_state": []}]
        # save the first spot which is s0 the current place of the drone 
        # spot is another name of s_list[0] so any changes will be seen in spot
        self.spot= self.neighbor_list[0]
        self.num_neigbors = 6
        for i in range(1, self.num_neigbors+1):
            s = {"name": "s" + str(i), "distance": 0, "priority": 0,"drones_in": 0,"drones_in_id":[] , "states": [], "previous_state": []}
            self.neighbor_list.append(s)
        
        # lance a thread to read messages continuously 
        self.xbee_receive_message_thread = threading.Thread(target=self.receive_message, args=(self,)) #pass the function reference and arguments separately to the Thread constructor.
        self.xbee_receive_message_thread.start()

    '''
    -------------------------------------------------------------------------------------
    ----------------------------- Communication ---------------------------------
    -------------------------------------------------------------------------------------
    '''
 
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

    def build_border_message(self, propagation_indicator ,target_ids, candidate):
    
        # Determine max byte count for numbers
        max_byte_count = max(
                            [self.determine_max_byte_size(num) for num in target_ids ]+
                            [self.determine_max_byte_size(num) for num in propagation_indicator]+
                            [self.determine_max_byte_size(candidate)]                            
                            )
            
        # Start message with 'F', followed by max byte count and then the length of the propagation_indicator
        message = Forming_border_header.encode() + struct.pack('>BB', max_byte_count, len(propagation_indicator))
    
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

        # End with '\n'
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

    def forward_border_message(self, propagation_indicator, targets_ids, candidate):
        ''' 
        The drone that receives the message will check target_ids, if it is included then it will forward the message 
        The message is forwarded to the neigbor drones that have ids are not in the recieved targets_ids and in the propagation_indicator
        Propagation_indicator will help to prevent the messages to have targets from behind
        If the drone is not in the targets then it will ignore the message 
        '''
        if self.id in targets_ids: # the current drone is in the targeted ids
            
            all_neigbors_id=[]
            for s in self.neighbor_list:
                all_neigbors_id.extend(s["drones_in_id"]) # add the id of all the niegbors including the current 
            
            new_propagation_indicator, new_targets_ids= self.calculate_propagation_indicator_target( propagation_indicator,targets_ids)
            msg= self.build_border_message(new_propagation_indicator,new_targets_ids, candidate) # as you see the candidate is resent as it was recived 
            self.send_msg(msg)
        
        else: # it is not in the tagreted ids then do nothing ( drop the message)
            return 
    
    def forward_broadcast_message(self,candidate):
        ''' 
        Build and send messgaes to all the niegbors and since only it is a broadcast 
        then propagation_indicator, targets_ids both are [-1].
        The most important is the candidate that fired the broadcast, which is already recieved from neigbors
        Note: since the message will be sent to all the drone around , but rememeber the ones that already received 
        it will not recieved it again and th reason is the flag that end the listener is raised and no reading of buffer will be performed
        '''
        msg= self.build_border_message([-1] ,[-1],candidate) # as you see the candidate is resent as it was recived 
        self.send_msg(msg)
        
    def send_msg(self,msg):
        #TODO deal with sending boradcasting 
        pass

    def send_demand(self): 
        # the drone will send message to demand the data
        # this mssage contains only way to ask data 
        # the message will be recived from all the drones are in the range of commuinication 
        pass 
    
    def retrive_msg_from_buffer(self):
        # device is the object of Xbee connection 
        xbee_message = device.read_data(0.02) #  0.02 timeout in seconds
        # read untile the bufere is empty or retrive 7 msgs
        msg=" "
        if xbee_message is not None:
            return msg  

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
    
    def build_spot_info_message(self):
        # Start message with 'E'
        message = Arrival_header.encode()

        # Encode positionX and positionY
        message += self.encode_float_to_int(self.positionX)
        message += self.encode_float_to_int(self.positionY)

        # Determine and append max byte count for self.id 
        max_byte_count = self.determine_max_byte_size(self.id)
        message += struct.pack('>B', max_byte_count)  # Note the single B format
        
        # Encode state 
        message += struct.pack('>B', self.state)

        # Append the id itself
        message += self.id.to_bytes(max_byte_count, 'big')

        # End with '\n'
        message += b'\n'

        return message

    def decode_spot_info_message(self,message):
        # Starting index after the header
        index = len(Arrival_header.encode()) # =1 
        
        # Decode positionX
        positionX_encoded = message[index:index+2] if len(message) - index == 6 else message[index:index+4]
        positionX = self.decode_int_to_float(positionX_encoded)
        index += len(positionX_encoded)
        
        # Decode positionY
        positionY_encoded = message[index:index+2] if len(message) - index == 6 else message[index:index+4]
        positionY = self.decode_int_to_float(positionY_encoded)
        index += len(positionY_encoded)
        
        # Decode state
        state = struct.unpack('>B', message[index:index+1])[0]
        index += 1

        # Decode max_byte_count for id
        max_byte_count = struct.unpack('>B', message[index:index+1])[0]
        index += 1
        
        # Decode self.id
        id_value = int.from_bytes(message[index:index+max_byte_count], 'big')

        return positionX, positionY, state, id_value

    def build_inherit_message(self,id_rec):
        # Start message with 'I'
        message = Inherit_header.encode() 
        
        max_byte_count = max([self.determine_max_byte_size(num) for num in self.rec_candidate]+ 
                             [self.determine_max_byte_size(id_rec)]+ 
                             [self.determine_max_byte_size(num) for num in self.rec_propagation_indicator]
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

    def circle_completed(self):
        if self.check_border_candidate_eligibility():
            self.change_state_to(Border)
            Broadcast_Msg= self.build_border_message([-1] ,[-1], self.id)
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
                self.forward_border_message(rec_propagation_indicator, target_ids, candidate) 

    def handel_broken_into_spot(self, msg):
        if self.border_candidate== True: 
            positionX, positionY, state, id_rec= self.decode_spot_info_message(msg)
            self.spot_info_update_neighbors_list(positionX, positionY, state, id_rec) # No need to mutex since the drone is in border_candidate only in it was Owner and reserved spot
            self.check_border_candidate_eligibility(observe=False) # use only the upddated list and see if the current drone still candidate 
            if self.border_candidate == False: # changed due to 6 neigbors filled 
                self.change_state_to(Free) # Has 6 neighbors                
                if not self.rec_candidate: # if the rec_candidate is not empty, means messages for border are already received 
                    msg=self.build_inherit_message(id_rec) # id_rec is the id of the drone hopped in 
                    self.send_msg(msg) 

    def handel_inheritence_message(self, msg):
        new_rec_candidate_values, new_rec_propagation_indicator_values, id_rec= self.decode_inherit_message(msg)
        if id_rec== self.id:
            self.update_rec_candidate(new_rec_candidate_values)
            self.update_rec_propagation_indicator(new_rec_propagation_indicator_values)

    def receive_message (self):
        self.Forming_Border_Broadcast_REC = threading.Event()
        
        while not self.Forming_Border_Broadcast_REC.is_set():
                      
            msg= self.retrive_msg_from_buffer() 
            time.slee(0.01) # wait to have msgs in the buffer

            msg= b'F\x01\x07\x01\x02\x03\x04\x05\x06\x07\x00\n' #example of message F [1, 2, 3, 4, 5, 6, 7] 0
            
            if msg.startswith(Expan_header.encode()) and msg.endswith(b'\n'):
                pass  
            
            elif msg.startswith(Arrival_header.encode()) and msg.endswith(b'\n'):
                self.handel_broken_into_spot(msg)
              
            elif msg.startswith(Inherit_header.encode()) and msg.endswith(b'\n'):
                self.handel_inheritence_message(msg)


            elif msg.startswith(Forming_border_header.encode()) and msg.endswith(b'\n'): # message starts with F end with \n 
                rec_propagation_indicator, target_ids, sender, candidate= self.decode_border_message(msg) 
                
                # End of the expansion broadcast msg
                if len(target_ids)==1 and target_ids[0]==-1 and rec_propagation_indicator[0]==-1 : 
                    if self.border_candidate==True: 
                        self.border_broadcast_respond(candidate)
                     
                    # Here any drone in any state needs to forward the boradcast message and rise ending flag  
                    self.forward_broadcast_message(candidate) 
                    self.Forming_Border_Broadcast_REC.set()

                
                elif self.id in  target_ids: # the drone respond only if it is targeted

                    if candidate == self.id: # the message recived contains the id of the drone means the message came back  
                        self.circle_completed ()  
                        
                    else: # the current drone received a message from a candidate border so it needs to forward it          
                        self.find_msg_direction_forward(rec_propagation_indicator,target_ids,sender,candidate )

                else: # Drone is not targeted ( doesnt matter it it is free or candidate) thus it drops the message 
                      # Do anything but wait for end of the expansion broadcast
                        continue 
                
            else: 
                print(" Undefined header")
    
    def start_observing(self):
        #send a demand 
        # set a timer 
        # recieve message 
        # send Ack 
        pass

    def clear_buffer(self):
        # read the buffer until it is empty 
        # while xbee_device.get_queue_length() > 0:
        #     xbee_device.read_data() 
        pass
       

    '''
    -------------------------------------------------------------------------------------
    -------------------------------- Movement calculation -------------------------------
    -------------------------------------------------------------------------------------
    '''
            
    # distance is _.xx 2 decimal
    def calculate_neighbors_distance_sink(self):
        
        DxDy2 = round((self.positionX * self.positionX) + (self.positionY * self.positionY),2)
        DxDy3a2 = round(DxDy2 + 3 * a * a,2)
        sqDx = round(sq3 * self.positionX,2)
        aDx = round((2*sq3) * self.positionX,2)
        Dy= round(self.positionY,2)

        #TODO you should consider a situation what inside the formaula is negative 
        for s in self.neighbor_list:
            formula = formula_dict.get(s["name"])
            if formula:
                distance = eval(formula, {'sqrt': sqrt, 'DxDy2': DxDy2, 'DxDy3a2': DxDy3a2, 'a': a, 'aDx': aDx, 'sqDx': sqDx, 'Dy': Dy})
                s["distance"] = round(distance,2)

        self.distance_from_sink=self.spot["distance"] # where spot is the data of s0 the current position 

    def check_num_drones_in_neigbors(self):
        #TODO it is about sending signal and recive it to count the number in each spot 
        # The idea the drone will recive signal from the nigboors the signal is coordinates 
        #of each drone and compare it, and since all the movement on 6 nigbor and the reive about a 
        # thn the messages usally are only from niegboor 
        # so it will compare with the posssible coordiantes of 
        # nigboor with respect to the sink( because can not compare with respect to the drone that recive the signal)
        # TODO in each of the spot s1-s6 creat x,y the coordinates with the respect to the sink then 
        # using x2+y2 find the distance 
            # no need for that because each drone will have it is own position calculated far from the sink 
        
        # here you need to use distance and compare it to know how to fill neighbor_list
        for s in self.neighbor_list:
            # for now i will have it from the Stdin 
            num_dron = int(input("\t Enter number of drone at "+s["name"]+" :"))
            s["drones_in"] = int(num_dron)
            # append the ids of each drone in the same spot to drones_in_id

    def findMinDistances_niegboor(self):
        min_distance = min(self.neighbor_list, key=lambda x: x["distance"])["distance"]
        self.min_distance_dicts =[s["name"] for s in self.neighbor_list if s["distance"] == min_distance]
        #self.min_distance_dicts = [s for s in self.neighbor_list if s["distance"] == min_distance]

    def set_priorities(self):
        denom= 4.0 * self.neighbor_list[0]["distance"] # 4*distance of s0 from sink 
        self.findMinDistances_niegboor()
        for s in self.neighbor_list:
            if s["drones_in"] == 0 : # free spot 
                s["priority"]= s["distance"]* C /denom
                #print("s["name"],s["priority"] )

            else: # s is occupied 
                if s["name"] in self.min_distance_dicts: #close to sink 
                    s["priority"]= float("inf") 
                else: # random float between [w*c+eps, w+1*c[
                    s["priority"]= random.uniform(s["drones_in"]* C + eps, (s["drones_in"]+1)*C)

    # find the neighbor have to go to and save it in direction_taken
    def find_priority(self): 
        min_Priority = min(self.neighbor_list, key=lambda x: x["priority"])["priority"]
        spot_to_go =[s["name"] for s in self.neighbor_list if s["priority"] == min_Priority]
        #print(int(self.Priority_to_go[0][1:])) #exteract only the number of nigboor  
        return int(spot_to_go[0][1:])

    def neighbors_election(self):
        # self.spot={"name": "s" + str(0), "distance": 0, "priority": 0, "drones_in": 1,"drones_in_id":[], "states": [] , "previous_state": 1, "phase": "E"}
        id_free = [id for id, state in zip(self.spot["drones_in_id"], self.spot["states"]) if state == Free]
        return min(id_free) # return the min id of a drone is in state Free 
    
    def is_it_Owner(self):
        self.check_num_drones_in_neigbors()
         #if s0 where the drone is conatins only the drone ( droen Owner) 
         # the done as you see count itself at the spot 
         # the drone should be Owner to be free 
        print("drone in s0", self.spot["drones_in"] )
        if self.spot["drones_in"]==1: # the drone is Owner
            self.change_state_to (Owner)
        print("drone state in s0", self.state )

    def convert_spot_angle_distance(self, dir):
        return DIR_VECTORS[dir][0], DIR_VECTORS[dir][1]
    
    def find_relative_spot(self, x, y ):
        for i, vector in enumerate(DIR_xy_distance_VECTORS):
            if abs(self.positionX - (x + vector[0])) < 1e-9 and abs(self.positionY - (y + vector[1])) < 1e-9:
                return i  # Return index if a match is found
        return -1  # Return -1 if no match is found
    
    
    '''
    -------------------------------------------------------------------------------------
    -------------------------------- Update upon movement--------------------------------
    -------------------------------------------------------------------------------------
    '''
    def change_state_to( self, new_state):
        self.previous_state= self.state # save the preivious state 
        self.state= new_state # change the state     

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

    # positionX , positionY are the coordinates from the sink 
    # format: _.xx 2 decimal
    def update_location(self, dir):
        x=  DIR_xy_distance_VECTORS[dir][0]
        y= DIR_xy_distance_VECTORS[dir][1]
        self.positionX =round(self.positionX + x,2) #add the value not assign because it is movement 
        self.positionY = round(self.positionY+ y ,2) #
    
    def update_candidate_spot_info_to_neighbors(self):
        msg= self.build_spot_info_message()
        self.send_msg(msg)   

    def spot_info_update_neighbors_list(self, positionX, positionY, state, id_rec):
        
        #Check if the id_rec of the drone is already in neighbor_list
        for s in self.neighbor_list:
            if id_rec in s["drones_in_id"]:

                # Find the index of the drone_id
                idx = s['drones_in_id'].index(id_rec)
                
                # Remove from drones_in_id and states
                s['drones_in_id'].pop(idx)
                s['states'].pop(idx)
                
                # Decrement drones_in count
                s['drones_in'] -= 1

        s_index= self.find_relative_spot(positionX, positionY)
        
        # Check if index is valid ( index between 0 and 6 )
        if 0 <= s_index <= self.num_neigbors+1:
            # Retrieve the corresponding spot
            s = self.neighbor_list[s_index] 

            # Append drone_id to drones_in_id and state_rec to states
            s['drones_in_id'].append(id_rec)
            s['states'].append(state)
            
            # Increment drones_in count
            s['drones_in'] += 1

        else:
            print("Invalid index provided")


    '''
    -------------------------------------------------------------------------------------
    --------------------------------- Forming the border---------------------------------
    -------------------------------------------------------------------------------------
    '''
    def count_element_occurrences(self):
        # Find the maximum element in the direction_taken list
        max_element = 7  # s0 to s6

        # Create a dictionary to store the frequencies, similar to hash map to count occurrences.
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
    
    # This function saves the spots that are occupied regarless if it is in the dominated direction or not 
    def save_occupied_spots(self):
        occupied_spots = [int(spot["name"][1:]) for spot in self.neighbor_list if spot["drones_in"] != 0]
        return occupied_spots
    
    def check_border_candidate_eligibility(self, observe=True):
        self.border_candidate=False  
        # Collect the info about the drones around , and that can be ignored in case of drone arriver after election
        if observe:
            self.check_num_drones_in_neigbors()

        self.border_neighbors = self.save_occupied_spots()
        self.dominated_direction= self.count_element_occurrences()
        # Define a dictionary to map directions to the corresponding spots_to_check_for_border values
        direction_to_check_map = {
            1: [1, 2, 6],
            2: [1, 2, 3],
            3: [3, 2, 4],
            4: [3, 4, 5],
            5: [4, 5, 6],
            6: [1, 5, 6]
        }
        self.spots_to_check_for_border=direction_to_check_map.get(self.dominated_direction, [])
        
        unoccupied_spots_counter = 0
        for check in self.spots_to_check_for_border:
            # Find the corresponding entry in neighbor_list by its name
            neighbor = next((n for n in self.neighbor_list if n["name"] == "s" + str(check)), None)
            if neighbor and (neighbor ["drones_in"] == 0 ): # spot also is not occupied 
                unoccupied_spots_counter += 1

        if unoccupied_spots_counter>0: # at least one spot is empty so the drone can be part of he border
            self.border_candidate=True
        
        return self.border_candidate

    def Fire_bordr_msg(self): 
        target_ids=[]
        for s in self.neighbor_list:
            target_ids.extend(s["drones_in_id"]) # add the id of all the niegbors including the current 
        # At the beginning  propagation_indicator and target_ids are the same in the source of the message 
        propagation_indicator= target_ids
        Msg= self.build_border_message(propagation_indicator, target_ids, self.id)
        self.send_msg(Msg)

    def Forme_border(self, vehicle):
       # since border_neighbors is used only for border drone then no need for it 
       #self.border_neighbors=[] #erase border_neighbors because no need for it 
        self.check_border_candidate_eligibility()
        
        # Ensure the drone is the only one in its spot, even if this check is done implicitly before in waiting time.

        while self.spot["drones_in"] > 1:
            wait_and_hover(vehicle, movement_time)
            self.check_num_drones_in_neigbors()
        
        if self.border_candidate :
            self.update_candidate_spot_info_to_neighbors() # Useful if the drone arrived and filled a spot made others sourounded 
            '''launch a message circulation for current candidat'''
            self.Fire_bordr_msg()

        else: 
            #means that the drone is sourounded in the expansion direction it can be set as free 
            self.change_state_to(Free)
            self.direction_taken=[] # reset the direction taken for the nex expansion 
        
        hover(vehicle) # Hovering while the forming border is done 

        # wait until the border procesdure is finished 
        self.Forming_Border_Broadcast_REC.wait()
        set_to_move(vehicle)
        self.Forming_Border_Broadcast_REC.clear() # reset for the next expansion 

    def search_for_target(self): # find if there is target in the area or not 
        
        if self.spot["disance"]==0 and self.state==Owner:  #the drone that is sink should be always itrremvable but it should be first Owner 
            self.change_state_to(Irremovable)

        # move int th lace and couver it to check if there is target or not 
    

    def first_exapnsion (self, vehicle):
        random_dir = int(random.randint(1, 6)) # 0 not include because it should not be in the sink
        self.direction_taken.append( random_dir)
        angle, distance = self.convert_spot_angle_distance(random_dir)
        move_body_PID(vehicle,angle, distance)
        calculate_relative_pos(vehicle)
        self.update_location(random_dir)

        while self.state !=Owner:
             #  after steady and hover 
                # start observing the location
            # in the new position find the distance of the neigboors 
            self.calculate_neighbors_distance_sink()
            print("checking for movement")
            self.check_num_drones_in_neigbors()
            self.set_priorities()

            # be carful it should not be move , it is set the psoition 
            # bcause move will use only the direction value as a movement from the current location 
            destination_spot= self.find_priority() 
            #if self.spot["drones_in"]>1: # more than one drone in the current spot
            elected_id= self.neighbors_election()
            if elected_id== self.id: # current drone is elected to move
                if destination_spot != 0: # it means movement otherwise it is hovering 
                    print ("go to S", destination_spot)
                    self.direction_taken.append( destination_spot)
                    angle, distance = self.convert_spot_angle_distance(destination_spot)
                    move_body_PID(vehicle,angle, distance)
                    self.update_location(destination_spot)
                    # Arrive to steady state and hover then start observing the location
                    wait_and_hover(vehicle, 5) 
            else: 
                wait_and_hover(vehicle,movement_time) # Wait untile the elected drone to leave to next stop.
                continue # do all the steps again escape update location because no movement done yet 
                '''Go to another iteration to redo all process because there is possibility that the spots around have changed'''

            calculate_relative_pos(vehicle)
            print("checking for update the state")
            self.is_it_Owner()
        
        # Before initiating the border procedure, it's important to wait for some time to ensures that the drone is alone in its spot.
        # This step eliminates the possibility of erroneously considering a drone as a border-candidate when another drone in the same spot is about to move.
        wait_and_hover(vehicle, sync_time) 

        self.Forme_border(vehicle)# will not return until the drones receive boradcast of forming border

        self.rec_propagation_indicator=[] # rest this indecator for the next iteration 
        self.xbee_receive_message_thread.join() # stop listening to message
        
        # Time guarantees that all drones begin the searching procedure simultaneously and synchronized.
        wait_and_hover(vehicle, sync_time) 

        self.search_for_target() # This is blocking until the end of movement 
              
        self.phase= Span_header
        
        # Since broadcast messages might still be circulating while retrieval has stopped, there could be leftover messages in the buffer.
        # It's essential to clear the buffer before the next phase to prevent any surplus.
        self.clear_buffer()