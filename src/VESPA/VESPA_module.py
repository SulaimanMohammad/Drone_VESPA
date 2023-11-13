from enum import Enum
from math import sqrt
import random
import copy
import struct
import sys
import os
import threading
import time
# Get the parent directory path
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
from drone.drone_ardupilot import *
from pathlib import Path
import re

'''
-------------------------------------------------------------------------------------
---------------------------------- Variables ----------------------------------------
-------------------------------------------------------------------------------------
'''
sq3=sqrt(3)
a=20
multiplier=100

'''Dictionary to hold the variables 
drones_number, id ,xbee_range, C, eps , speed_of_drone,  movement_time= a*speed_of_drone*(1+ 0.2)  
scanning_time, sync_time, multiplier, defined_groundspeed '''
global_vars = {}

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
    global DIR_VECTORS
    DIR_VECTORS = [
    [0, 0],    # s0 // don't move, stay
    [90, a],   # s1
    [30, a],   # s2
    [330, a],  # s3
    [270, a],  # s4
    [210, a],  # s5
    [150, a]   # s6
    ]

def set_a(val):
    ''' a* 0.95 aim for a buffer zone. This ensures that even if there's a slight miscalculation in the range or
    a sudden change in environmental conditions, there's some leeway before communication is lost.'''
    global a
    a= val*0.95
    update_DIR_xy_distance_VECTORS() # changing of a should change the direction distances

#Return the byte count necessary to represent max ID.
def determine_max_byte_size (number):
    if number == 0:
        return 1  # Handle zero explicitly
    elif number <= 0xFF:
        return 1
    elif number <= 0xFFFF:
        return 2
    elif number <= 0xFFFFFF:
        return 3
    else:
        return 4

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
Movement_command= 'M'
Calibration= 'C'
Demand_header= 'D'
Response_header= 'R'
# Expansion Headers
Expan_header= 'E'
Arrival_header= 'A'
Inherit_header= 'I'
Forming_border_header= 'F'
#Spanning Headers
Spanning_header= 'S'
Target_coordinates_header= 'T'
# Blancing Headers
Local_balance_header='L'
Guidance_header= 'G'
Balance_header= 'B'
Algorithm_termination_header="O"

# Create an array of headers
headers = [
    Movement_command, Calibration, Demand_header, Response_header,
    Expan_header, Arrival_header, Inherit_header, Forming_border_header,
    Spanning_header, Target_coordinates_header, Local_balance_header,
    Guidance_header, Balance_header, Algorithm_termination_header
]

# Generate an array of ASCII values to be used in retiveing messages 
headers_ascii_values = [ord(header) for header in headers]

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
        self.hight=z
        self.state=None
        self.previous_state=1
        self.drone_id_to_sink=0
        self.drone_id_to_border=0
        self.a=None
        self.id=None
        self.target_detected= False
        self.border_candidate=False
        self.dominated_direction=0
        self.phase= Expan_header
        self.spots_to_check_for_border=[]
        self.rec_candidate=[] # contains ids of drone that fired a messaging circle
        self.drone_id_to_sink=[]
        self.drone_id_to_border=[]
        self. min_distance_dicts=[] # nigboor close to the sink
        self.allowed_spots=[]    # contains the spots that are not occupied while forming the border
        self.direction_taken=[]  # direction path (spots) that are taken in the phase
        self.neighbor_list = []  # list that contains the 6 neighbors around the current location
        self.rec_propagation_indicator=[]
        self.elected_id=None
        self.read_vars_from_file() # Include set id and xbee range "a" 
        # Set the variables globally
        globals().update(global_vars)
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
        
        self.current_target_ids=[]
        self.lock_state = threading.Lock()
        self.lock_neighbor_list = threading.Lock()
        self.neighbors_list_updated = threading.Event()
        self.forming_border_msg_recived= threading.Event()
        self.VESPA_termination= threading.Event() 
        self.elected_droen_arrived= None    
        self.Forming_Border_Broadcast_REC= None
        self.start_expanding= None
        self.end_of_balancin= None

        
        

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
        while msg.startswith(Response_header.encode()):
            positionX, positionY, state, previous_state,id_value= self.decode_spot_info_message(msg)
            self.update_neighbors_list(positionX, positionY, state, previous_state,id_value)
            self.calculate_neighbors_distance_sink()
            msg= self.retrive_msg_from_buffer()
        self.neighbors_list_updated.set()
        
        return msg # since the message does not start with reponse then it should be treated in another way 

    def exchange_neighbors_info_communication(self,msg):
        # Receiving message asking for data 
        if msg.startswith(Demand_header.encode()) and msg.endswith(b'\n'):
            data_msg= self.build_spot_info_message(Response_header) # Build message that contains all data
            self.send_msg(data_msg)

        # Receiving message containing data     
        if msg.startswith(Response_header.encode()) and msg.endswith(b'\n'):
            new_msg= self.get_neighbors_info()
        
        return new_msg 
            


    def send_msg(self,msg):
        #TODO deal with sending boradcasting
        pass

    def retrive_msg_from_buffer(self):
        time.sleep(0.05) # wait to have msgs in the buffer
        # device is the object of Xbee connection
        xbee_message = device.read_data(0.02) #  0.02 timeout in seconds
        # read untile the bufere is empty or retrive 7 msgs
        msg=" "
        if xbee_message is not None:
            return msg
        else:
            return None

    def encode_float_to_int(self, value, precision= multiplier):
        """Encodes a float as an integer with a given multiplier."""
        encoded = int(value * precision)
        # Choose the format based on the size of the integer
        if encoded <= 65535:
            return struct.pack('>BH', 2, encoded)  # Length 2 bytes, value
        else:
            return struct.pack('>BI', 4, encoded)  # Length 4 bytes, value


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
        max_byte_count = determine_max_byte_size(self.id)
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
    def calculate_neighbors_distance_sink(self):
        DxDy2 = round((self.positionX * self.positionX) + (self.positionY * self.positionY),2)
        DxDy3a2 = round(DxDy2 + 3 * a * a,2)
        sqDx = round(sq3 * self.positionX,2)
        aDx = round((2*sq3) * self.positionX,2)
        Dy= round(self.positionY,2)
        for s in self.neighbor_list:
            formula = formula_dict.get(s["name"])
            if formula:
                distance = eval(formula, {'sqrt': sqrt, 'DxDy2': DxDy2, 'DxDy3a2': DxDy3a2, 'a': a, 'aDx': aDx, 'sqDx': sqDx, 'Dy': Dy})
                s["distance"] = round(distance,2)
        self.distance_from_sink=self.spot["distance"] # where spot is the data of s0 the current position


    def read_vars_from_file(self):
        # Current script directory
        current_dir = Path(__file__).resolve().parent
        # Path to the parent directory
        parent_dir = current_dir.parent
        global Operational_Data_path
        Operational_Data_path = parent_dir/'Operational_Data.txt'

        with open(Operational_Data_path, 'r') as file:
            for line in file:
                match = re.match(r"(\w+)\s*=\s*([^#]+)", line)
                if match:
                    var_name, value = match.groups()
                    if var_name in ["max_acceleration", "max_deceleration"]:
                        continue
                    if var_name == "id":
                        self.id = int(value.strip())  # Set id to the object's attribute
                    elif var_name == "xbee_range":
                        set_a(int(value.strip()))  # Use set_a() for xbee_range
                    else:
                        try:
                            global_vars[var_name] = eval(value.strip(), {}, global_vars)
                        except NameError:
                            global_vars[var_name] = value.strip()
            
    def update_xbee_range(self,new_a):
        new_content = []
        with open(Operational_Data_path, 'r') as file:
            for line in file:
                if line.startswith('xbee_range'):
                    new_content.append(f'xbee_range = {new_a}\n')
                else:
                    new_content.append(line)

        # Write the updated content back to the file
        with open(Operational_Data_path, 'w') as file:
            file.writelines(new_content)
 
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
         # If s0 where the drone exist conatins only the drone (droen Owner)
        print("drone in s0", self.spot["drones_in"] )
        if self.spot["drones_in"]==1: # the drone is Owner
            self.change_state_to (Owner)

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

    def update_location(self, dir):
        # PositionX , positionY are the coordinates from the sink
        # Format: _.xx 2 decimal
        x=  DIR_xy_distance_VECTORS[dir][0]
        y= DIR_xy_distance_VECTORS[dir][1]
        self.positionX =round(self.positionX + x,2) # Add the value not assign because it is movement
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

    def update_state_in_neighbors_list( self, id, state):
        # Change information about the state of one of neighbors
        # Lock to avoid any data race since there are two threads ( main , xbee)
        with self.lock_neighbor_list:
            for s in self.neighbor_list:
                if id in s["drones_in_id"]:
                    # Find the index of the drone_id and changr the state
                    idx = s['drones_in_id'].index(id)
                    s['states'][idx]= state
    
    def check_termination(self):
        # if VESPA_termination is set that means the algorithm is done 
        # This should be used in while loop where VESPA is applied when this flag is not set 
        if self.VESPA_termination.is_set():
            return False
        else: 
            return True

    '''
    -------------------------------------------------------------------------------------
    -------------------------------- Movement -------------------------------
    -------------------------------------------------------------------------------------
    '''
    def convert_spot_angle_distance(self, dir):
        return DIR_VECTORS[dir][0], DIR_VECTORS[dir][1]

    def find_relative_spot(self, x, y ):
        for i, vector in enumerate(DIR_xy_distance_VECTORS):
            if abs(self.positionX - (x + vector[0])) < 1e-9 and abs(self.positionY - (y + vector[1])) < 1e-9:
                return i  # Return index if a match is found
        return -1  # Return -1 if no match is found

    def move_to_spot(self,vehicle, destination_spot):
        set_to_move(vehicle)
        self.direction_taken.append( destination_spot)
        angle, distance = self.convert_spot_angle_distance(destination_spot)
        try:
            move_body_PID(vehicle,angle, distance)
        except Exception as e:
            print(f"An error occurred: {str(e)}")
            vehicle.mode = VehicleMode ("RTL")
            vehicle.close()
        self.update_location(destination_spot)
        self.clear_buffer() # need to clear the bufer from message received on the road
        # Arrive to steady state and hover then start observing the location
        hover(vehicle)

    def search_for_target(self): # find if there is target in the area or not
        # move in the place and couver it to check if there is target or not
        self.target_detected= True
        if self.state == Border:
            self.change_state_to(Irremovable_boarder)
        else: 
            self.change_state_to(Irremovable)

    def return_home(self, vehicle):
        if self.id==0: # Sink
            time.sleep(10) 
            vehicle.mode = VehicleMode ("LAND")
        else:
            vehicle.mode = VehicleMode ("RTL")
        vehicle.close() 

