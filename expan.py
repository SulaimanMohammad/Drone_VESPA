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
movement_time= a*speed_of_drone (1+ 0.2)  # add 20% of time for safty 
scanning_time= 10 # in second ( TODO function to the speed and size of a )

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

Alone=0
Free=1
Border=2
Irremovable= 3 
Irremovable_boarder=4 

Expan_header= "E"
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
        self.border_messaging_circle_completed=False 
        self.dominated_direction=0
        self.phase= Expan_header
        self.spots_to_check_for_border=[]
        self.drone_id_to_sink
        self.drone_id_to_border
        self. min_distance_dicts=[] # nigboor close to the sink 
        self.border_neighbors=[] # contains the spots that are occupied while forming the border
        self.direction_taken=[]  # direction path (spots) that are taken in the phase 
        self.neighbor_list = []  # list that contains the 6 neighbors around the current location
        # init s0 and it will be part of the spots list 
        self.neighbor_list=[{"name": "s" + str(0), "distance": 0, "priority": 0, "drones_in": 1,"drones_in_id":[], "states": [] , "previous_state": []}]
        # save the first spot which is s0 the current place of the drone 
        # spot is another name of s_list[0] so any changes will be seen in spot
        self.spot= self.neighbor_list[0]
        self.num_neigbors = 6
        for i in range(1, self.num_neigbors+1):
            s = {"name": "s" + str(i), "distance": 0, "priority": 0,"drones_in": 0,"drones_in_id":[] , "states": [], "previous_state": []}
            self.neighbor_list.append(s)

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

    def build_border_message(self, target_ids):
    
        # Determine max byte count for numbers
        max_byte_count = max([self.determine_max_byte_size(num) for num in target_ids + [self.id]])

        # Start message with 'F', followed by max byte count and then the length of the numbers list
        # Forming_border_header.encode() = b'F'
        message = Forming_border_header.encode() + struct.pack('>BB', max_byte_count, len(target_ids))

        # Add each number to the message using determined byte count
        for num in target_ids:
            message += num.to_bytes(max_byte_count, 'big')

        # Append the last number using the determined byte count
        message += self.id.to_bytes(max_byte_count, 'big')

        # End with '\n'
        message += b'\n'
        
        return message


    def decode_message(message):
        # Check that the message starts with "F" and ends with "\n"
        if not message.startswith(Forming_border_header.encode()) or not message.endswith(b'\n'):
            raise ValueError("Invalid message format")

        # Extract max byte count and length of numbers list
        max_byte_count, numbers_length = struct.unpack('>BB', message[1:3])

        # Determine the start and end indices for the main list of numbers
        indices = [(i * max_byte_count + 3, (i + 1) * max_byte_count + 3) for i in range(numbers_length)]

        # Extract the numbers
        targets_ids = [int.from_bytes(message[start:end], 'big') for start, end in indices]

        # Extract the last number
        last_number_start = 3 + max_byte_count * numbers_length
        sender_candidate = int.from_bytes(message[last_number_start:-1], 'big')

        return targets_ids, sender_candidate


    def send_demand(self): 
        # the drone will send message to demand the data
        # this mssage contains only way to ask data 
        # the message will be recived from all the drones are in the range of commuinication 
        pass 

    def receive_message (self):
        # TODO this sould be listening all the time with while loop 
        # here you need to receive the message but you need to know when to start the algo of expansion 
        # the drone will have a timer will start after the demand message is sent 
        # after the receive of each message the timer will be reset 
        # after the timer is up means no more message is arriving 
        # you should notice that is important to use timer because we can not use counter of message because spots might be unoccipied 
        # The moment the drone receive the message beside reseting the time a ACK message will be sent 
        # the recived message will contain the id that sent the message, the ACK will contain that id 
        # if that drone received the ACK will not try to resend message
          # self.send_ACK() 
          # 
        # the recive function should change also the variable, and deal with demand and ACK from other drones    
        
        msg_header="E"
        id_rec= 0
        if msg_header== "E":
            pass
        elif msg_header== "F":
            if id_rec == self.id: # the message recived contains the id of the drone means the message came back  
                pass
    
    def send_ACK(self):
        pass

    def start_observing(self):
        #send a demand 
        # set a timer 
        # recieve message 
        # send Ack 
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

    
    def is_it_alone(self):
        self.check_num_drones_in_neigbors()
         #if s0 where the drone is conatins only the drone ( droen alone) 
         # the done as you see count itself at the spot 
         # the drone should be alone to be free 
        print("drone in s0", self.spot["drones_in"] )
        if self.spot["drones_in"]==1: # the drone is alone
            self.change_state_to (Alone)
        print("drone state in s0", self.state )


    def convert_spot_angle_distance(self, dir):
        return DIR_VECTORS[dir][0], DIR_VECTORS[dir][1]
    

    '''
    -------------------------------------------------------------------------------------
    -------------------------------- Update upon movement--------------------------------
    -------------------------------------------------------------------------------------
    '''
    def change_state_to( self, new_state):
        self.previous_state= self.state # save the preivious state 
        self.state= new_state # change the state     

    # positionX , positionY are the distance from the sink 
    def update_location(self, dir):
        x=  DIR_xy_distance_VECTORS[dir][0]
        y= DIR_xy_distance_VECTORS[dir][1]
        self.positionX =self.positionX + x#DIR_VECTORS[dir][0]# add the value not assign because it is movement 
        self.positionY = self.positionY+ y #DIR_VECTORS[dir][1]
        #return [self.positionX, self.positionY] 


    def update_state(self):
        # if self.state=="Alone" : # the drone is alone so see if it is free or border or irrmovable
        if self.spot["disance"]==0 and self.state==Alone:  #the drone that is sink should be always itrremvable but it should be first alone 
            self.change_state_to(Irremovable)

        while self.state==Alone:
            counter=0
            # need to check the number arouund because many spot was changed meanwhile 
            for s in self.neighbor_list:
                if s["drones_in"] >=1: 
                    counter= counter +1 
                    continue
                else: 
                    break
            print("counter", counter)
            if counter==7: # all neighboor are occupied including the drone itself 
                self.change_state_to(Free)
                # since border_neighbors is used only for border drone then no need for it 
                self.border_neighbors=[] #erase border_neighbors because no need for it 

            else: #drone is alone but not sourrounded by drones
                self.check_border()
            #TODO border just if it doent have niegboor on th path of the expansion 
            # NOTE :  after yann he said it is not possible to have more than one drone
            # here i need to verfiy whhat is should be 
            # because if it is border then that means it will do deal_state 



    '''
    -------------------------------------------------------------------------------------
    --------------------------------- Forming the border---------------------------------
    -------------------------------------------------------------------------------------
    '''
    def count_element_occurrences(self):
        # Find the maximum element in the direction_taken list
        max_element = 7  # s0 to s6

        # Create a dictionary to store the frequencies, Python dictionaries (similar to hash maps) to count occurrences.
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
    
    
    def save_occupied_spots(self):
        occupied_spots = [int(spot["name"][1:]) for spot in self.neighbor_list if spot["drones_in"] != 0]
        return occupied_spots
    
    # start_messaging_circle this message will return only when it meet the goal 
    def start_messaging_circle(self):
        # forming the target_ids that should recive the message that is related to forming the border 
        # it consisit sending to all the neigbors around 
        target_ids=[]
        for s in self.neighbor_list:
            target_ids.extend(s["drones_in_id"]) # add the id of all the niegbors including the current 

        Msg= self.build_border_message(target_ids)
        self.send_msg(Msg)
        
        # the headrer is F representing forming border
        # the massage will contain the ID of the drone that started the circle 
        # Always keep reading and reciving 
        # in the reciver check for F header 
            # firt the drone will ckeck the id of the msg is the same of the current drone 
                # then return and send brodcast 
            # else the msg id is not same of the current drone 
            # the drone will check if it is self.border_candidate=True it will forward the msg and keep the id of the sender 
                # if the drone is not self.border_candidate=True then it will drop the message 

        # the drone will become bordere if recived the good message and it is self.border_candidate=True 
            # and it needs again to check for border unoccupied ,and send a bordcast 
            # otherwise check agin for free
            # in reciving part also 
        # The drones those are in situation as candidates to be part of the border and receive the broadcast message with the ID they compare that ID with the one they saved  
        #  in step 2 which mean that they where part of the circle, so they check that they still meet the requirements and change to border state. 
        # this can reduce the amount of messaging. and ensure that any drone was in the completed circle of communication is a part of the border
        # the broadcast should be taged as brodcast to differeniate between brodcast msg and fprming border msg 


    def check_border(self):
        self.border_candidate=False  
        # the drone after it is candidateq should continue searcching anif not should search to be free 
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
            self.start_messaging_circle() 
            if self.border_messaging_circle_completed: 
                self.change_state_to( Border)
        else: 
            #means that the drone is sourounded in the expansion direction it can be set as free 
            self.change_state_to(Free)
            self.direction_taken=[] # reset the direction taken for the nex expansion 

    

    def search_for_target(self): # find if there is target in the area or not 
        pass
        # move int th lace and couver it to check if there is target or not 
    


    def first_exapnsion (self, vehicle):
        random_dir = int(random.randint(1, 6)) # 0 not include because it should not be in the sink
        self.direction_taken.append( random_dir)
        angle, distance = self.convert_spot_angle_distance(random_dir)
        move_body_PID(vehicle,angle, distance)
        calculate_relative_pos(vehicle)
        self.update_location(random_dir)

        while self.state !=Alone:
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
                    # after steady and hover 
                    # start observing the location
                    # after arriving ( send )
            else: # another should move 
                time.sleep (movement_time) # Wait untile the elected drone to leave to next stop.
                # TODO here sleep means loiter 
                continue # do all the steps again escape update location because no movement done yet 
                '''The need of re-doing all process because there is possibility that the spots around have changed'''

            calculate_relative_pos(vehicle)
            print("checking for update the state")
            self.is_it_alone()

        self.check_border()
        # check border should not return until the drones receive bordcast of forming border 
        # the drone is alone and can search for border or become Free 
        self.update_state() # it inclueds forming the border
        self.search_for_target() # find if there is target in the area or not 
        #the drone will never do to the second phase before finihing the search and the update 
        # because it will be problem if all not in the same phase 
        self.spot["phase"]= "S"
        #TODO here the drones should wait in loop until reciving a brodcast of finishing the expanshion 