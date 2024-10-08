from math import sqrt
import random
import struct
import sys
import os
import threading
import time
import copy
# Get the parent directory path
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
from drone.drone_ardupilot import *
from .headers_variables import * 
import signal
import subprocess


'''
-------------------------------------------------------------------------------------
---------------------------------- Variables ----------------------------------------
-------------------------------------------------------------------------------------
'''
set_env(globals())

sq3=sqrt(3)
a=xbee_range*xbee_proficiency_ratio
multiplier=100
effective_a= ((int)(a / sq3 * 1000) / 1000.0)
'''Dictionary to hold the variables 
drones_number, id ,xbee_range, C, eps , scanning_time, sync_time, multiplier, defined_groundspeed '''
global_vars = {}
global DIR_xy_distance_VECTORS
def update_DIR_xy_distance_VECTORS():
    global DIR_xy_distance_VECTORS
    DIR_xy_distance_VECTORS = [
    [0, 0],                                         # s0 // don't move, stay
    [a, 0],                                         # s1
    [(1.0 / 2.0) * a, (3.0 / 2.0) * effective_a],   # s2
    [-(1.0 / 2.0) * a, (3.0 / 2.0) * effective_a],  # s3
    [-(1 * a), 0],                                  # s4
    [-(1.0 / 2.0) * a, -(3.0/ 2.0) * effective_a],  # s5
    [(1.0 / 2.0) * a, -(3.0 / 2.0) * effective_a]   # s6
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
    a= val
    update_DIR_xy_distance_VECTORS() # changing of a should change the direction distances

#Return the byte count necessary to represent max ID.
def determine_max_byte_size(number):
    if number< 0:
        return (number.bit_length() + 7) // 8
    else:
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
    "s1": "sqrt(DxDy3a2 + effective_a * aDx)",
    "s2": "sqrt(DxDy3a2 + effective_a * (sqDx + (3 * Dy)))",
    "s3": "sqrt(DxDy3a2 + effective_a * (3 * Dy - sqDx))",
    "s4": "sqrt(DxDy3a2 - (aDx*effective_a))",
    "s5": "sqrt(DxDy3a2 - effective_a * (sqDx + (3 * Dy)))",
    "s6": "sqrt(DxDy3a2 - effective_a * (3 * Dy - sqDx))"
}

Owner=0
Free=1
Border=2
Irremovable= 3
Irremovable_boarder=4

'''
-------------------------------------------------------------------------------------
----------------------------- Calss members, init__ ---------------------------------
-------------------------------------------------------------------------------------
'''
class Drone:
    def __init__(self,x,y,z, passed_id=None):
        self.positionX=x
        self.positionY=y
        self.distance_from_sink=0 # the distance of the drone from  the sink
        self.hight=z
        self.state=Free
        self.previous_state=Free
        self.drone_id_to_sink=0
        self.drone_id_to_border=0
        self.a=None
        self.target_detected= False
        self.border_candidate=False
        self.phase= Expan_header
        self.spots_to_check_for_border=[]
        self.neighbors_ids=[] # contains ids of all neighbors drones 
        self.rec_candidate=[] # contains ids of drone that fired a messaging circle
        self.drone_id_to_sink=[]
        self.drone_id_to_border=[]
        self. min_distance_dicts=[] # nigboor close to the sink
        # allowed_spots contains the spots that are not occupied while forming the border
        # At the beginning all allowed and that is needed for first expansion 
        self.allowed_spots=[0,1,2,3,4,5,6]   
        self.neighbor_list = []  # list that contains the 6 neighbors around the current location
        self.elected_id=None
        self.destination_spot=0
        set_a(a)
        if passed_id==None: # if it is not passed as argument use id in Opertional_ data that will has id of each RPI
            self.id=drone_id
        else:
            self.id= passed_id
        # Alt for each drone related to ID to avoid collision while movement
        self.ref_alt= round(sqrt(pow(a*ref_alt_ratio,2)- pow(((a*ref_alt_ratio)/sq3),2)),2) # Reference alt (drone hight when it is alone )
        self.drone_alt= ((self.id-1)*spacing)+ self.ref_alt # Drone alt while movement
        self.defined_groundspeed= defined_groundspeed
        # init s0 and it will be part of the spots list
        self.neighbor_list=[{"name": "s" + str(0), "distance": 0, "priority": 0, "drones_in": 1,"drones_in_id":[], "states": [] , "previous_state": []}]
        # save the first spot which is s0 the current place of the drone
        # spot is another name of self.neighbor_list[0] so any changes will be seen in spot
        self.current_spot= self.neighbor_list[0]
        self.current_spot["drones_in_id"].append(self.id)
        self.current_spot["states"].append(self.state)
        self.current_spot["previous_state"].append(self.previous_state)
        self.num_neigbors = 6
        for i in range(1, self.num_neigbors+1):
            s = {"name": "s" + str(i), "distance": 0, "priority": 0,"drones_in": 0,"drones_in_id":[] , "states": [], "previous_state": []}
            self.neighbor_list.append(s)
        
        # Identification 
        self.collect_drones_info_timer_lock = threading.Lock()
        self.collected_ids=[] # The list that contains all the drones that participate in VESPA
        self.sink_handshake= threading.Event()
        self.broadcasted_Identificatio=[]
        self.broadcasted_sink_handshake=[]
        self.estimated_number_drones=0
        # First movement messages 
        self.first_movement_command_broadcasted=[]
        self.first_movement_command_received= False
        self.start_expanding= threading.Event()
        self.current_target_id=None
        self.current_target_id_lock=threading.Lock() 
        self.lock_state = threading.Lock()
        self.lock_neighbor_list = threading.Lock()
        self.lock_demanders_timer =threading.Lock()
        self.demanders_received_data = threading.Event()
        self.VESPA_termination= threading.Event() 
        self.Forming_Border_Broadcast_REC= threading.Event()
        self.Forming_border_failed= threading.Event() # Flag used to stop the threads of forming border in case the failure of formation 
        self.demanders_list=[]
        self.demand_timer=None
        self.remaining_time_demand=None
        self.remaining_time_resposnse=None
        self.resposnse_rec_counter=0
        # Event refer if exchanging messages done or not, clear measn in progress, set = done 
        self.list_finished_update= threading.Event()
        self.list_finished_update.set() # Should be set so the first access is guaranteed and there after access will be cleared ( reading in progress)
        self.last_seen_demand_neighbors_info=0
        self.demander_lock=threading.Lock()
        self.lock_boder_timer =threading.Lock() 

        self.candidate_to_send=[] # list saves the candidate to send thier msg and confirm arriving 
        self.candidate_to_send_lock= threading.Lock()
        self.border_formed= True #  Used to loop many times trying to forme the border when the border successfully formed adn no need to try more 
        self.movemnt_from_border=False # used to detect first movement to reset allowed spot
        self.neighbor_list_upon_border_formation=[] # Contains the toplogy around upon forming the border 
        self.border_verified=threading.Event()

        self.Emergency_stop = threading.Event() # Flag in case a problem or abort is needed
        self.expansion_stop=  threading.Event() # Flage refer that the expansion is ended 

        if uart:
            connect_xbee(xbee_serial_port, baud_rate)
        else:
            connect_xbee(Tx,Rx, baud_rate)

    '''
    -------------------------------------------------------------------------------------
    ---------------- Timers and managment of Demand response-----------------------------
    -------------------------------------------------------------------------------------
    '''

    def append_id_demanders_list(self,value):
        if value not in self.demanders_list:
            self.demanders_list.append(value)

    def remove_id_demanders_list(self,value):
        while value in  self.demanders_list:
             self.demanders_list.remove(value)   

    def initialize_timer_demand(self):
        self.reset_timer_demand()
        while (not self.Emergency_stop.is_set()): # It should be while true but the Emergency should be taken into account 
            with self.lock_demanders_timer:
                self.remaining_time_demand -= 0.1
                if self.remaining_time_demand <= 0:
                        break
            time.sleep(0.1)

    def reset_timer_demand(self):
        with self.lock_demanders_timer:
            self.remaining_time_demand=2*exchange_data_latency

    def initialize_timer_resposnse(self):
        self.reset_timer_resposnse()
        self.resposnse_rec_counter=0
        while (not self.Emergency_stop.is_set()):
            self.remaining_time_resposnse -= 0.1
            if self.remaining_time_resposnse <= 0:
                    break
            time.sleep(0.1)

    def reset_timer_resposnse(self):
        self.remaining_time_resposnse=3*exchange_data_latency

    def resend_data(self):
        if not self.demanders_received_data.is_set():
            data_msg= self.build_spot_info_message(Response_header) 
            send_msg(data_msg)
        self.demanders_received_data.clear()
            
    '''
    -------------------------------------------------------------------------------------
    ---------------------------------- Communication ------------------------------------
    -------------------------------------------------------------------------------------
    '''
    def build_drone_is_ready_message(self):
        message= Prepared_header.encode()
        max_byte_count = determine_max_byte_size(self.id)
        message += struct.pack('>B', max_byte_count)
        message += self.id.to_bytes(max_byte_count, 'big')
        message += b'\n'
        return message
    
    def decode_drone_is_ready_message(self,msg):
        demand_header = msg[:1].decode()
        # Extract the max_byte_count which is the next byte after the header
        max_byte_count = struct.unpack('>B', msg[1:2])[0]
        # Extract the ID using the max_byte_count
        id_start_index = 2
        id_end_index = id_start_index + max_byte_count
        id_bytes = msg[id_start_index:id_end_index]
        ids = int.from_bytes(id_bytes, 'big')
        return ids
    
    def decode_Start_VESPA_message(self, encoded_message):
        header_size = 1  
        demand_header = encoded_message[:header_size].decode()
        # Extract the max_byte_count which is the next byte after the header
        max_byte_count = struct.unpack('>B', encoded_message[header_size:header_size+1])[0]
        # Extract the ID using the max_byte_count
        num_drone_start_index = header_size + 1
        num_drone_end_index = num_drone_start_index + max_byte_count
        num_drone_bytes = encoded_message[num_drone_start_index:num_drone_end_index]
        num_drone = int.from_bytes(num_drone_bytes, 'big')
        return num_drone

    def build_emergency_message(self):
        message= Emergecy_header.encode()
        message += b'\n'
        return message
    
    def position_sensitive_checksum(self, message):
        checksum = 0
        for index, byte in enumerate(message):
            # Multiply each byte by its position index (position + 1 to avoid multiplication by zero)
            checksum += (index + 1) * byte
        return checksum % 256
    
    def build_data_demand_message(self):
        message= Demand_header.encode()
        max_byte_count = determine_max_byte_size(self.id)
        message += struct.pack('>B', max_byte_count)
        message += self.id.to_bytes(max_byte_count, 'big')
        message += b'\n'
        return message
    
    def decode_data_demand_message(self,encoded_message):
        header_size = 1  
        demand_header = encoded_message[:header_size].decode()
        # Extract the max_byte_count which is the next byte after the header
        max_byte_count = struct.unpack('>B', encoded_message[header_size:header_size+1])[0]
        # Extract the ID using the max_byte_count
        id_start_index = header_size + 1
        id_end_index = id_start_index + max_byte_count
        id_bytes = encoded_message[id_start_index:id_end_index]
        ids = int.from_bytes(id_bytes, 'big')
        return ids
    

    def compare_with_neighbor_list(self, list1, key):
        for item1, item2 in zip(sorted( self.neighbor_list , key=lambda x: x['name']), sorted(list1, key=lambda x: x['name'])):
            if item1[key] != item2[key]:
                return False
        return True    
    
    def demand_neighbors_info(self): 
        # This function will be called by many threads and since it contains reset of data, and need to finish receiving data so list updated
        # So the function should not be called until it is completly finished or the list will be wrong if 2 threads called it at the same time  
        # Only one thread should ask the demand and the rest wait
        if self.list_finished_update.is_set() : # There is no other thread doing the update   
            self.list_finished_update.clear()
            copy_neighbor_list= copy.deepcopy(self.neighbor_list) # use deepcopy or it will be reference not copy 
            recollect_data=0
            self.rest_neighbor_list()
            reseted_neighbor_list= copy.deepcopy(self.neighbor_list) 
            
            while (self.compare_with_neighbor_list(reseted_neighbor_list,'drones_in')) and (recollect_data<2) and (not self.Emergency_stop.is_set()): 
                demand_msg= self.build_data_demand_message()
                send_msg(demand_msg)
                self.initialize_timer_resposnse()
                recollect_data= recollect_data +1
                time.sleep(exchange_data_latency)

            if self.resposnse_rec_counter==0: # No response recieved so it is blocked thread restor the old list 
                with self.lock_neighbor_list:
                    self.neighbor_list=  copy.deepcopy(copy_neighbor_list) 

            self.list_finished_update.set()
        else:
            self.list_finished_update.wait() # If 2 thread called thi function, one will execute it and the seconf will wait for the result 

    def build_ACK_data_message(self, target_id):
        message= ACK_header.encode()
        max_byte_count = max([determine_max_byte_size(self.id)]+
                            [determine_max_byte_size(target_id)])
        message += struct.pack('>B', max_byte_count)
        message += self.id.to_bytes(max_byte_count, 'big')
        message+=target_id.to_bytes(max_byte_count, 'big')
        message += b'\n'
        return message
    
    def decode_ACK_data_message(self, encoded_message):
        # Skip the header (1 byte)
        content = encoded_message[1:]
        # Extract the max_byte_count (1 byte)
        max_byte_count = struct.unpack('>B', content[:1])[0]
        # Calculate the start and end positions for id and target_id
        id_start = 1
        id_end = id_start + max_byte_count
        target_id_start = id_end
        target_id_end = target_id_start + max_byte_count
        # Extract id and target_id
        id_bytes = content[id_start:id_end]
        target_id_bytes = content[target_id_start:target_id_end]
        ids = int.from_bytes(id_bytes, 'big')
        target_id = int.from_bytes(target_id_bytes, 'big')
        return ids, target_id
    
    def encode_float_to_int(self, value, precision=multiplier):
        """Encodes a float as an integer with a given multiplier."""
        encoded = int(value * precision)
        # Choose the format based on the size of the integer
        if -32768 <= encoded <= 32767:  # Range of a signed short
            return struct.pack('>bh', 2, encoded)  # Length 2 bytes, value
        else:
            return struct.pack('>bi', 4, encoded)  # Length 4 bytes, value

    def decode_int_to_float(self, encoded_float):
        # Check the byte length to decide the format
        if len(encoded_float) == 2:
            value = struct.unpack('>h', encoded_float)[0]  # 'h' for signed short
        elif len(encoded_float) == 4:
            value = struct.unpack('>i', encoded_float)[0]  # 'i' for signed int
        # Convert back to float
        return value / multiplier  

    def build_spot_info_message(self, header):
        message = header.encode()
        # Encode positionX and positionY
        message += self.encode_float_to_int(self.positionX)
        message += self.encode_float_to_int(self.positionY)
        # Encode state
        message += struct.pack('>B',self.get_state())
        message += struct.pack('>B', self.get_previous_state())
        # Determine and append max byte count for self.id
        max_byte_count = determine_max_byte_size(self.id)
        # Append the id 
        message += struct.pack('>B', max_byte_count)        
        message += self.id.to_bytes(max_byte_count, 'big')
        message +=b'\n'
        return message

    def decode_spot_info_message(self,message):
        index = 1  # Header is 1 byte
        # Decode length and positionX
        lengthX = struct.unpack('>B', message[index:index+1])[0]
        index += 1
        positionX_encoded = message[index:index+lengthX]
        positionX = self.decode_int_to_float(positionX_encoded)
        index += lengthX
        # Decode length and positionY
        lengthY = struct.unpack('>B', message[index:index+1])[0]
        index += 1
        positionY_encoded = message[index:index+lengthY]
        positionY = self.decode_int_to_float(positionY_encoded)
        index += lengthY
        # Decode state and previous_state
        state = struct.unpack('>B', message[index:index+1])[0]
        index += 1
        previous_state = struct.unpack('>B', message[index:index+1])[0]
        index += 1
        # Read byte for max_byte_count
        max_byte_count = struct.unpack('>B', message[index:index+1])[0]
        index += 1
        # Decode the id
        id_value = int.from_bytes(message[index:index+max_byte_count], 'big')
        index += max_byte_count
        return [positionX, positionY, state, previous_state, id_value]
    
    ''' 
        Procedures to treat the incoming response answer 
    '''
    def get_neighbors_info(self,msg):
        decoded_msg= self.decode_spot_info_message(msg) # If the message is invalid decode will return [-1]
        # Ack will be sent only if the message is correct, Otherwise the lack of ACK will force the target of resending data 
        if len(decoded_msg)>1 : # No erorr of receiving
            positionX, positionY, state, previous_state, id_value= decoded_msg
            if self.remaining_time_resposnse: # timer already initialized
                self.reset_timer_resposnse()
                self.resposnse_rec_counter=self.resposnse_rec_counter+1
            Ack_msg=self.build_ACK_data_message(id_value)
            send_msg(Ack_msg)
            self.update_neighbors_list(positionX, positionY, state, previous_state,id_value)
        
    def collect_demands(self):
        try: 
            # wait for request , wait all requests and send only once 
            self.initialize_timer_demand()
            if self.demanders_list: # there are drone demaneded 
                data_msg= self.build_spot_info_message(Response_header) # Build message that contains all data
                send_msg(data_msg)
                # wait Ack from the demander and in case not all recived re-send
                with self.demander_lock:
                    self.demanders_list=[]
                self.initialize_timer_demand()
                self.resend_data()
        except:
            write_log_message("Thread collect_demands Interrupt received, stopping...")
            self.emergency_stop()   
        
    def exchange_neighbors_info_communication(self,msg):
        # Receiving message asking for data 
        if msg.startswith(Demand_header.encode()) and msg.endswith(b'\n'):
            id_need_data= self.decode_data_demand_message(msg)
            if not self.demanders_list: # empty demander then launch the thread of timer 
                self.demand_timer = threading.Thread(target=self.collect_demands, args=()) #pass the function reference and arguments separately to the Thread constructor.
                self.demand_timer.start()  
            with self.demander_lock: 
                self.append_id_demanders_list(id_need_data)
            self.reset_timer_demand()
                
        elif msg.startswith(ACK_header.encode()) and msg.endswith(b'\n'):
                sender_id, target_id =self.decode_ACK_data_message(msg)
                if target_id== self.id and sender_id in self.demanders_list: 
                    self.remove_id_demanders_list(sender_id)
                    if self.remaining_time_demand: # timer already initialized 
                        self.reset_timer_demand()
                if len(self.demanders_list)==0:
                    self.demanders_received_data.set()
                else:
                    self.demanders_received_data.clear()
                if self.demand_timer is not None and self.demand_timer.is_alive() and self.remaining_time_demand<=0: 
                    self.demand_timer.join()

        # Receiving message containing data     
        elif msg.startswith(Response_header.encode()) and msg.endswith(b'\n'):
            self.get_neighbors_info(msg)

        elif msg.startswith(Handover_change_header.encode()) and msg.endswith(b'\n'):
            decoded_msg= self.decode_spot_info_message(msg) # If the message is invalid decode will return [-1]
            # Ack will be sent only if the message is correct, Otherwise the lack of ACK will force the target of resending data 
            if len(decoded_msg)>1 : # No erorr of receiving
                positionX, positionY, state, previous_state, id_value= decoded_msg
                self.update_neighbors_list(positionX, positionY, state, previous_state,id_value)

    def system_is_ready(self):
        #Send message to GCS that system is ready 
        msg=self.build_drone_is_ready_message()
        send_msg(msg)

      
    '''
    -------------------------------------------------------------------------------------
    -----------Send data message to the GCS (ground controle station )-------------------
    -------------------------------------------------------------------------------------
    '''

    def build_data_message(self,id_target, ids, longitude, latitude, data):
        # Convert numbers to string
        id_target_str = str(id_target).encode()
        id_str = str(ids).encode()
        longitude_str = str(longitude).encode()
        latitude_str = str(latitude).encode()
        data_send = str(data).encode()
        message = Info_header.encode() + b',' + id_target_str + b',' + id_str + b',' + longitude_str + b',' + latitude_str + b',' + data_send + b'\n'
        return message

    def decode_data_message(self, message):
        # Remove header and terminal
        content = message[len(Info_header)+1:-1]
        # Split by the comma to get floats
        parts = content.split(b',')
        id_target = int(parts[0])
        sender_id = int(parts[1])
        longitude = float(parts[2])
        latitude = float(parts[3])
        data = float(parts[4])
        return id_target, sender_id, longitude, latitude, data 
    

    def find_close_to_sink(self):
        '''
        1.Remove the First Element and Filter the List
        Start by removing the first element from the list.
        Then, filter the remaining elements to include only those where drones_in is greater than 0.
        
        2.Find the Minimum Distance:
        Determine the Result Based on Minimum Distance:

        If there is only one element with the minimum distance and drones_in equals 1, return the number in drones_in_id.
        If there is only one element with the minimum distance and drones_in is greater than 1, select the smallest number from drones_in_id.
        If multiple elements have the same minimum distance, return the smallest number from all the drones_in_id values across those elements.
                
        '''
        neighbor_list=self.get_neighbor_list()
        # Step 1: Remove the first element and filter by drones_in > 0
        filtered_list = [item for item in neighbor_list[1:] if item.get("drones_in", 0) > 0]

        # Step 2: Find the minimum distance
        min_distance = min(item["distance"] for item in filtered_list)

        # Step 3: Determine the result based on the scenarios
        min_distance_items = [item for item in filtered_list if item["distance"] == min_distance]

        if len(min_distance_items) == 1:
            # Case: Exactly one minimum distance found
            min_item = min_distance_items[0]
            if min_item["drones_in"] == 1:
                # Case: drones_in == 1, return drones_in_id directly
                result = min_item["drones_in_id"]
            else:
                # Case: drones_in > 1, choose smallest drones_in_id
                result = min(min_item["drones_in_id"])
        else:
            # Case: Multiple items with the minimum distance, choose smallest drones_in_id
            all_ids = [id for item in min_distance_items for id in item["drones_in_id"]]
            result = min(all_ids)
        return result[0]
    
    def forward_data_message(self,msg, id_to_send_to=None):
        id_target, sender_id, longitude,  latitude, data= self.decode_data_message(msg) 
        # Only the targeted drone will respond           
        if self.id == id_target:
            if id_to_send_to== None: 
                if self.id==1:# It is sink send it to station 
                    id_to_send_to= 0 #send to station
                else: 
                    id_to_send_to= self.find_close_to_sink() 
            # only reforward the data recived change only to what drone to send to 
            msg= self.build_data_message(id_to_send_to, sender_id , longitude, latitude, data)
            send_msg(msg)

    def send_data_message_station(self, vehicle, id_to_send_to=None,  data=None):
        #Get current GPS data 
        if check_gps_fix(vehicle): 
            current_lon = vehicle.location.global_relative_frame.lon
            current_lat = vehicle.location.global_relative_frame.lat
        else: 
            current_lon=0
            current_lat=0

        # Chose to what drone to send the message 
        if id_to_send_to== None:      
            if self.id==1:
                # Sink will send to GCS 
                id_to_send_to=0
            else:   
                # Other drone will find the drone closest to the sink 
                id_to_send_to= self.find_close_to_sink()
        
        if data== None:
            data_to_send=0
        else:
            data_to_send=data
            
        msg= self.build_data_message(id_to_send_to, self.id , current_lon, current_lat, data_to_send)
        send_msg(msg)
        time.sleep(10)
    

    '''
    -------------------------------------------------------------------------------------
    -------------------------------- Update upon movement--------------------------------
    -------------------------------------------------------------------------------------
    '''
    
    def all_neighbor_spots_owned(self):
        for neighbor in self.get_neighbor_list():
            num_drones_in = neighbor["drones_in"]
            states = neighbor["states"]
            if num_drones_in >0 and not all( (state == Owner or state== Irremovable or state==Irremovable_boarder ) for state in states):
                return False
        return True
    
    def calculate_neighbors_distance_sink(self):
        DxDy2 = ((self.positionX * self.positionX) + (self.positionY * self.positionY))
        DxDy3a2 = (DxDy2 + 3 * effective_a * effective_a)
        sqDx = (sq3 * self.positionX)
        aDx = ((2*sq3) * self.positionX)
        Dy= (self.positionY)
        with self.lock_neighbor_list: # Writting need a lock 
            for s in self.neighbor_list:
                formula = formula_dict.get(s["name"])
                if formula:
                    distance = eval(formula, {'sqrt': sqrt, 'DxDy2': DxDy2, 'DxDy3a2': DxDy3a2, 'effective_a': effective_a, 'aDx': aDx, 'sqDx': sqDx, 'Dy': Dy})
                    s["distance"] = round(distance,2)
            # Note that current_spot accesed directly no need to use get_cuurent_spot since the lock is already used above ( or it will be deadlock)
            self.distance_from_sink=self.current_spot["distance"] # where spot is the data of s0 the current position

    # This function is used for the drone elected and move so it will rearrange the old list corresponding to the movement 
    def rearrange_neighbor_statically_upon_movement(self,moveTOspot):
        moving_drone_id=self.id
        # Define the movement rules
        movement_rules = {
            1: {'s0': 's4', 's1': 's0', 's2': 's3', 's6': 's5', 'empty': ['s4', 's5', 's6']},
            2: {'s0': 's5', 's1': 's6', 's2': 's0', 's3': 's4', 'empty': ['s4', 's5', 's6']},
            3: {'s0': 's6', 's3': 's0', 's4': 's5', 's2': 's1', 'empty': ['s1', 's5', 's6']},
            4: {'s0': 's1', 's4': 's0', 's3': 's2', 's5': 's6', 'empty': ['s1', 's2', 's6']},
            5: {'s0': 's2', 's4': 's3', 's5': 's0', 's6': 's1', 'empty': ['s1', 's2', 's3']},
            6: {'s0': 's3', 's1': 's2', 's5': 's4', 's6': 's0', 'empty': ['s2', 's3', 's4']}
        }
        with self.lock_neighbor_list:
            # Create a dictionary for faster spot lookup
            spot_lookup = {spot['name']: spot for spot in self.neighbor_list}

            # Empty the specified spots
            for spot_name in movement_rules[moveTOspot]['empty']:
                spot_lookup[spot_name].update({'drones_in': 0, 'drones_in_id': [], 'states': [], 'previous_state': []})

            # Move the drones based on movement rules, excluding the 'empty' key
            for spot_name, new_spot_name in {k: v for k, v in movement_rules[moveTOspot].items() if k != 'empty'}.items():
                spot = spot_lookup[spot_name]
                new_spot = spot_lookup[new_spot_name]

                # Special handling for s0 and the moving drone
                if spot_name == 's0' and moving_drone_id in spot['drones_in_id']:
                    drone_index = spot['drones_in_id'].index(moving_drone_id)
                    moving_drone_states = spot['states'][drone_index]
                    moving_drone_previous_state = spot['previous_state'][drone_index]

                    # Move other drones
                    drones_to_move = [d_id for d_id in spot['drones_in_id'] if d_id != moving_drone_id]
                    states_to_move = [state for i, state in enumerate(spot['states']) if i != drone_index]
                    prev_states_to_move = [state for i, state in enumerate(spot['previous_state']) if i != drone_index]

                    new_spot['drones_in_id'].extend(drones_to_move)
                    new_spot['states'].extend(states_to_move)
                    new_spot['previous_state'].extend(prev_states_to_move)

                    # Update s0 to only have the moving drone
                    spot['drones_in_id'] = [moving_drone_id]
                    spot['states'] = [moving_drone_states]
                    spot['previous_state'] = [moving_drone_previous_state]
                    spot['drones_in'] = 1
                else:
                    # Move all drones for other spots
                    new_spot['drones_in_id'].extend(spot['drones_in_id'])
                    new_spot['states'].extend(spot['states'])
                    new_spot['previous_state'].extend(spot['previous_state'])

                    spot['drones_in_id'] = []
                    spot['states'] = []
                    spot['previous_state'] = []
                    spot['drones_in'] = 0
                # Update the 'drones_in' count for the new spot
                new_spot['drones_in'] = len(new_spot['drones_in_id'])    

    def rearrange_neighbor_statically_upon_elected_arrival (self,elected_drone_id, target_spot):
        target_spot_name = f's{target_spot}'
        source_spot = None
        drone_states = []
        drone_previous_state = []
        with self.lock_neighbor_list:
            # Find and update source spot
            for spot in self.neighbor_list:
                if elected_drone_id in spot['drones_in_id']:
                    source_spot = spot
                    drone_index = spot['drones_in_id'].index(elected_drone_id)
                    drone_states = spot['states'][drone_index]
                    drone_previous_state = spot['previous_state'][drone_index]

                    # Update source spot
                    spot['drones_in'] -= 1
                    spot['drones_in_id'].remove(elected_drone_id)
                    del spot['states'][drone_index]
                    del spot['previous_state'][drone_index]
                    break

            # Update target spot
            for spot in self.neighbor_list:
                if spot['name'] == target_spot_name:
                    spot['drones_in'] += 1
                    spot['drones_in_id'].append(elected_drone_id)
                    spot['states'].append(drone_states)
                    spot['previous_state'].append(drone_previous_state)
                    break
        self.check_Ownership()
        
    def update_xbee_range(self,new_a):
          # Current script directory
        current_dir = Path(__file__).resolve().parent
        # Path to the parent directory
        parent_dir = current_dir.parent
        global Operational_Data_path
        Operational_Data_path = parent_dir/'Operational_Data.txt'
        
        new_content = []
        with open(Operational_Data_path, 'r') as file:
            for line in file:
                if line.startswith('xbee_range'):
                    new_content.append(f'xbee_range = {new_a}\n')
                else:
                    new_content.append(line)
        set_a(new_a)
        # Write the updated content back to the file
        with open(Operational_Data_path, 'w') as file:
            file.writelines(new_content)

    def inform_neighbors_of_change(self):
        msg=self.build_spot_info_message(Handover_change_header)
        send_msg(msg)

    def get_neighbor_list(self):
        # To get the list,if it is in the process of updating should wait the process or we get wrong list 
        if not self.list_finished_update.is_set():
            self.list_finished_update.wait()
        with self.lock_neighbor_list: # only one thread can access 
            return self.neighbor_list
    
    # To read state, thread-safe
    def get_state(self):
        with self.lock_state:
            return self.state
   
    def get_previous_state(self):
        with self.lock_state:
            return self.previous_state
    
    def get_current_spot(self):
        with self.lock_neighbor_list:
            return self.current_spot
            
    def write_state(self, state):
        with self.lock_state:
            self.state= state
        self.inform_neighbors_of_change()

    def change_state_to( self, new_state):
        with self.lock_state:
            self.previous_state= self.state # save the preivious state
            self.state= new_state # change the state
        with self.lock_neighbor_list:
            # Change state in neighbor_list where first entry is the drone spot
            self.current_spot["previous_state"][0]=self.previous_state
            self.current_spot["states"][0]=self.state
        self.inform_neighbors_of_change()

    def check_Ownership(self):
        if self.get_state() != Owner:
            if self.get_current_spot()["drones_in"]==1: # the drone is alone 
                self.change_state_to (Owner)
            # Many drone on the spot but non is owner( if drones arrived to same spot at same time)
            # Leader election with min id is chosen as owner 
            elif self.get_current_spot()["drones_in"]>1:
                # Check if there is  owner or (irremovable or border because they are considered as owner) then it can not be owner of spot 
                non_free = all(state != Free for state in self.get_current_spot()["states"])
                if non_free==0: # Only free there
                    all_free = all(state == Free for state in self.get_current_spot()["states"])
                    if all_free:
                        min_id = min(self.get_current_spot()["drones_in_id"])
                        write_log_message ( "\n chosed from many in same point:")
                        write_log_message(f"min_id= {min_id}" )
                        # current drone is chosen
                        if  min_id == self.id:
                            self.change_state_to (Owner)
                        else:
                            min_id_index =  self.get_current_spot()["drones_in_id"].index(min_id)
                            with self.lock_neighbor_list: # change state of entry in the current spot
                                self.current_spot["states"][min_id_index]=Owner

    
    def correct_states_after_comm(self):
        # If another spot contains only one drone but did not change yet it is state then do it here 
        # That can happen if the drone is still in movement and did not arrive yet but its corrdiantes were set to destination 
        for s in self.get_neighbor_list()[1:]: 
            if s["drones_in"]==1:
                for i, state in enumerate (s["states"]):
                    if state != Owner:
                        s["states"][i]=Owner
                        s["previous_state"][i]=Free 

                    
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
        if dir !=0: # Update only if the position changed
            x=  DIR_xy_distance_VECTORS[dir][0]
            y= DIR_xy_distance_VECTORS[dir][1]
            self.positionX =round(self.positionX + x,2) # Add the value not assign because it is movement
            self.positionY = round(self.positionY+ y ,2)
            # self.rearrange_neighbor_statically_upon_movement(dir)
            # Find the distance of the neigboors at the new position
            self.calculate_neighbors_distance_sink()
            self.inform_neighbors_of_change()

    def rest_neighbor_list(self):
        with self.lock_neighbor_list: # Locker needed since it is writing 
            for neighbor in self.neighbor_list:
                if 'drones_in_id' in neighbor and self.id in neighbor['drones_in_id']:
                    # Find the index of the target_id in drones_in_id
                    index = neighbor['drones_in_id'].index(self.id )
                    # Keep the target_id and corresponding states and previous_states
                    neighbor['drones_in_id'] = [self.id ]
                    neighbor['states'] = [neighbor['states'][index]] if index < len(neighbor['states']) else []
                    neighbor['previous_state'] = [neighbor['previous_state'][index]] if index < len(neighbor['previous_state']) else []
                    neighbor['drones_in'] = 1  # Reduce drones_in by removing all but the target_id
                else:
                    # For neighbors not containing the target_id, clear drones_in_id and adjust drones_in accordingly
                    neighbor['drones_in_id'] = []
                    neighbor['drones_in'] = 0
                    # Ensure 'states' and 'previous_state' are properly formatted even if empty
                    if 'states' in neighbor:
                        neighbor['states'] = []
                    if 'previous_state' in neighbor:
                        neighbor['previous_state'] = []

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
                            idx = s['drones_in_id'].index(id_rec)
                            s['states'][idx]= state
                            s['previous_state'][idx]=previous_state
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
            # Receive signal from drone out of the 6 neighbors
            # If the drone was in neighbors and left hen it should be removed 
            with self.lock_neighbor_list:
                for s in self.neighbor_list:
                    if id_rec in s["drones_in_id"]:
                        idx = s['drones_in_id'].index(id_rec)
                        s['drones_in_id'].pop(idx)
                        s['states'].pop(idx)
                        s['previous_state'].pop(idx)
                        s['drones_in'] -= 1

        self.neighbors_ids=[] 
        '''
        Don't use self.get_neighbor_list() because it contains locks, 
        and update_neighbors_list is called within the exchange_data procedure which will cause a wrong answer due to waiting for lock 
        which is released after the timer is up calling it here will block the list until the lock is released  which results in wrong communication 
        '''
        with self.lock_neighbor_list:
            for s in self.neighbor_list:
                self.neighbors_ids.extend(s['drones_in_id']) # save  all the ids in all spots 


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
    ---------------------------------- Movement -----------------------------------------
    -------------------------------------------------------------------------------------
    '''
    def take_off_drone(self, vehicle):
        # FLy with GPS
        try:
            if check_gps_fix(vehicle) and use_GPS:
                arm_and_takeoff(vehicle,self.drone_alt)
                hover(vehicle) # Ensure that the drone stay in plac
            else:
                #TODO in drone_ardupilot.py in drone repo # arm_and_takeoff_no_GPS(vehicle,self.drone_alt)
                pass
        except:
            write_log_message("Could not take off ")
            self.emergency_stop() 

    def simple_goto_thread(self, vehicle, lon, lat):
            set_to_move(vehicle)
            try: 
                point1 = LocationGlobalRelative(lat,lon ,self.drone_alt)
                vehicle.simple_goto( point1, groundspeed=defined_groundspeed) # Non-blocking movement need sleep to wait it to be done 
                time.sleep((a/defined_groundspeed)+10) # Wait the movement to be done
            except:
                write_log_message("An error occurred while move with simple_goto")
                self.emergency_stop() 
            hover(vehicle) # Ensure that the drone stay in place 

    def move_using_coord(self, vehicle, lon, lat):
            '''
            simple_goto is non blocking movement so it will retuen after the command is sent, thus you need to sleep to give the drone time to move
            and Can't use sleep to wait arriving because this function called in listener which block the listener.
            Lunching a thread to move will help to move and wait the arrivale without blocking the listener
            '''
            move_thread = threading.Thread(target= self.simple_goto_thread, args=(vehicle, lon, lat))
            move_thread.start()

    def convert_spot_angle_distance(self, dir):
        return DIR_VECTORS[dir][0], DIR_VECTORS[dir][1]

    def find_relative_spot(self, x, y, tolerance=1): # tolerence in m 
        # Calculate the difference
        dx = x - self.positionX
        dy = y - self.positionY
        # Iterate through the vectors to find a matching one within the tolerance
        for i, vector in enumerate(DIR_xy_distance_VECTORS):
            if abs(vector[0] - dx) <= tolerance and abs(vector[1] - dy) <= tolerance:
                return i  
        return -1


    def calculate_velocity_based_on_alt_differences(self,vehicle, destination_spot):
        # The current drone knows the ids of the drones in the destination spot so it can estimate there altitude
        # The function will see if there is enough gap so the drone get inside with high speed or should moce slowly
        destination_spot_info= self.get_neighbor_list()[destination_spot]
        movement_velocity=speed_of_drone
        if destination_spot_info["drones_in"] >0: # The spot is not empty 
            alitudes_diff=[]
            for drone_id in destination_spot_info['drones_in_id']:
                drone_in_spot_altitude= ((drone_id-1)*spacing)+ self.ref_alt
                alitudes_diff.append( abs(drone_in_spot_altitude- get_altitude(vehicle))) # estimated alt of drones - acutal hight of current drone 
            write_log_message(f"Before movement the alitudes_diff with other drones is {alitudes_diff}" )
            if all(alts > 1.8*spacing for alts in alitudes_diff):
                movement_velocity=speed_of_drone # can go in speed , there is enough place to move in 
            else: #risky gap should move slowly 
                movement_velocity=speed_of_drone-1  
        else: # no drones in the spot 
                movement_velocity=speed_of_drone 

        return movement_velocity 
    

    def move_to_spot(self,vehicle, destination_spot):
        go_to_altitude(vehicle,self.drone_alt) # Be sure that you are on the right alt to move 
        movement_velocity= self.calculate_velocity_based_on_alt_differences(vehicle, destination_spot)
        self.update_location(destination_spot)
        set_to_move(vehicle)
        angle, distance = self.convert_spot_angle_distance(destination_spot)
        try:
            move_body_PID(vehicle,angle, distance, self.Emergency_stop,self.ref_alt,max_velocity=movement_velocity )
        except:
            write_log_message("An error occurred while move_body_PID")
            self.emergency_stop()
        # Arrive to steady state and hover then start observing the location
        hover(vehicle)

    def search_for_target(self): # find if there is target in the area or not
        pass 
        # TODO implement the methode to detect if the drone find target ( like Computer vision) with movement in the Hexagon of the drone 
        # move in the place and couver it to check if there is target or not
        if self.target_detected== True:
            if self.get_state() == Border:
                self.change_state_to(Irremovable_boarder)
            else: 
                self.change_state_to(Irremovable)

    '''
    -------------------------------------------------------------------------------------
    ------------------------------- Emergency procedures --------------------------------
    -------------------------------------------------------------------------------------
    '''

    def return_home(self, vehicle):
        if vehicle.armed and (get_altitude(vehicle) >= 1): # Drone in the sky 
            if self.id==1: # Sink
                time.sleep(10) 
                vehicle.mode = VehicleMode ("LAND")
            else:
                time.sleep((a/defined_groundspeed)*(self.id)) # Wait time proportional to the id and xbee range( distance between spots) so not all back to home at the same time 
                vehicle.mode = VehicleMode ("RTL")
        
    def set_thread_flags(self):
        self.expansion_stop.set()
        self.list_finished_update.set() # All thread that aiting end of gathering data should stop or we have deadlock 
        self.demanders_received_data.set() # Mark that all demander got answer to avoid blocking on resending data where other drones stopped listening upon emergency 
        self.VESPA_termination.set()

    def interrupt(self, vehicle):
        if (not self.Emergency_stop.is_set()):
            self.Emergency_stop.set()
            self.set_thread_flags()
            emergency_msg= self.build_emergency_message()
            send_msg(emergency_msg)
            self.stop_wifi()
            write_log_message("Retuen home")
            vehicle.remove_attribute_listener('velocity', on_velocity)
            self.return_home(vehicle)
            time.sleep(exchange_data_latency)
            vehicle.close()
            close_xbee_port()
            os._exit(0)  # Exit the program with a non-zero status
 
            
    def emergency_stop(self):
        write_log_message("Emergency stop detected. Exiting function.")
        if (not self.Emergency_stop.is_set()):
            # brodcast it again
            emergency_msg= self.build_emergency_message()
            send_msg(emergency_msg)
            write_log_message("Call interrupt")
            os.kill(os.getpid(), signal.SIGINT) # That will call interrupt which use vehicle object to return home  

    def check_VESPA_safety(self,vehicle):
        '''
        If the companion computer (e.g., Raspberry Pi) suddenly stops while the drones are in the air, 
        the service will automatically restart upon reboot and run the main program. 
        The main program will first check if the drone is at altitude and armed, indicating that the Raspberry Pi stopped during the operation due to a fault. 
        If this is the case, an emergency return command will be sent to the drones, as the main program cannot restart the process normally without reapplying all the phases.
        '''
        #TODO Read the recent log and get the old statue and in what phase the drone was and demand info of drones around and build neighbor list again 
        if get_altitude(vehicle)>2 or vehicle.armed:
            write_log_message("Safety failure, Drone is armed and in sky -> RTL")
            #return immediately because in case the communication is not activated so emergency stop will not be executed will and that risk not set RTL
            vehicle.mode = VehicleMode ("RTL") 
            self.emergency_stop()

    '''
    -------------------------------------------------------------------------------------
    -------------------------------- Operations at Spot ---------------------------------
    -------------------------------------------------------------------------------------
    '''

    def count_num_people(self,scan_time=30,rssi_threshold=-70):
        try:
            # Define the path to the Bash script
            parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '../'))
            script_path = os.path.join(parent_directory, 'Estimate_num_people', 'detect_wifi.sh')

            # Ensure the script is executable
            subprocess.run(['chmod', '+x', script_path], check=True)

            # Call the Bash script 
            result = subprocess.run([script_path, str(scan_time), str(rssi_threshold)], 
                                    capture_output=True, text=True, check=True)
            
            # Extract and return the number of phones
            number_of_phones = result.stdout.strip()
            return float(number_of_phones)
        
        except:
            write_log_message("Could not count people ")
            self.emergency_stop() 


    def stop_wifi(self):
        parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '../'))
        script_path = os.path.join(parent_directory, 'Estimate_num_people', 'bring_down_wifi.sh')
        # Ensure the script is executable
        subprocess.run(['chmod', '+x', script_path], check=True)
        # Call the Bash script 
        result = subprocess.run(['sudo',script_path], capture_output=True, text=True, check=True)
