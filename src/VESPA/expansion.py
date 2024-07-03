from .VESPA_module import *
from .form_border_one_direction import *

set_env(globals())

'''
-------------------------------------------------------------------------------------
---------------------------------- Communication ------------------------------------
-------------------------------------------------------------------------------------
'''
def build_identification_message(header, id_sent):
    message = header.encode()
    max_byte_count = determine_max_byte_size(id_sent)
    message += struct.pack('>B', max_byte_count)
    message += id_sent.to_bytes(max_byte_count, 'big')
    message += b'\n'
    return message

def decode_identification_message(encoded_message):
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

def build_movement_command_message(id, spot, float1, float2):
    # Movement messages are encoded using string beause they are done only once, efficiency accepted
    # Convert numbers to string and encode
    id_str= str(id).encode()
    s_str=str(spot).encode()
    float1_str = str(float1).encode()
    float2_str = str(float2).encode()
    # Construct message
    message = Movement_command.encode() + id_str +b','+s_str+ b','+ float1_str + b',' + float2_str + b'\n'
    return message

def decode_movement_command_message(message):
    # Remove header and terminal
    content = message[1:-1]
    # Split by the comma to get floats
    parts = content.split(b',')
    ids = int(parts[0])
    s = int(parts[1])
    float1 = float(parts[2])
    float2 = float(parts[3])
    return ids, s, float1, float2

def build_calibration_message(indicator, xbee_range):
    # Encode numbers
    num1_encoded = struct.pack('b', indicator)  # signed byte
    num2_encoded = struct.pack('B', xbee_range)  # unsigned byte
    # Construct message
    message = Calibration.encode + num1_encoded + num2_encoded + b'\n'
    return message

def decode_calibration_message(message):
    # Extract the numbers
    indicator = struct.unpack('b', message[1:2])[0]
    xbee_range = struct.unpack('B', message[2:3])[0]
    return indicator, xbee_range

def build_expan_elected(id):
    # Determine max byte count for numbers
    max_byte_count = determine_max_byte_size(id)
    # Encode the header and the max byte count
    message = Expan_header.encode() + struct.pack('>B', max_byte_count)
    # Pack `id` based on max_byte_count
    message += id.to_bytes(max_byte_count, byteorder='big')
    message += b'\n'
    return message
   
def decode_expan_elected(message): 
    # Extract the max byte count
    max_byte_count = struct.unpack('>B', message[1:2])[0]
    # Extract the id using the appropriate byte count
    id_start = 2
    id_end = id_start + max_byte_count
    elected_id = int.from_bytes(message[id_start:id_end], byteorder='big')
    return elected_id

def handel_elected_drone_arrivale(self,msg):
    rec_id=decode_expan_elected(msg)
    if rec_id==self.elected_id:
        self.elected_droen_arrived.set()

def check_continuity_of_listening(self):
    if (not self.expansion_stop.is_set()) and (not self.Emergency_stop.is_set()):
        return True
    else: 
        return False

def expansion_listener (self,vehicle):

    while check_continuity_of_listening(self):
        try:
            '''
            Data-Driven retrieve_msg_from_buffer will continue reading and will break only wwhen data is available 
            in this way the listener will process the message upon arrival 
            For that, the outer loop will continue until the listener stops which is controlled by the flag and 
            break when data is available and the next iteration of 
            the listener will recall this loop again to trigger the listener only when data is available 
            '''
            msg= retrieve_msg_from_buffer(self.expansion_stop)
            
            self.exchange_neighbors_info_communication(msg)
            
            if msg.startswith(Emergecy_header.encode()) and msg.endswith(b'\n'):
                self.emergency_stop()
                break

            elif msg.startswith(Identification_header.encode()) and msg.endswith(b'\n'):
                if self.id==1: # It is sink drone, check if the id of the drone is not saved if not save and send confirmation 
                    update_initial_drones_around(self,msg)

            elif msg.startswith(Identification_Caught_header.encode()) and msg.endswith(b'\n'):
                ids=decode_identification_message(msg)
                if self.id==ids: # Message from the sink recognizes that the identification is arrived 
                    self.sink_handshake.set() 

            elif msg.startswith(Movement_command.encode()) and msg.endswith(b'\n'):
                ids, spot, lon, lat= decode_movement_command_message(msg)
                if ids==-1 and spot==-1 and lon==0 and lat==0: # mean all drone are in sky
                    self.start_expanding.set()
                else:
                    initial_movement(self, vehicle,ids, spot, lon, lat)

            elif msg.startswith(Calibration.encode()) and msg.endswith("\n"):
                calibration_ping_pong(self, vehicle, msg )

            elif msg.startswith(Expan_header.encode()) and msg.endswith(b'\n'):
                handel_elected_drone_arrivale(self, msg)
                
            elif msg.startswith(Forming_border_header.encode()) and msg.endswith(b'\n'): # message starts with F end with \n
                form_border_one_direction(self,Forming_border_header,msg)
                #form_border_two_direction(self,Forming_border_header,msg)
            elif msg.startswith(Verify_border_header.encode()) and msg.endswith(b'\n'):
                verify_border(self,Verify_border_header,msg)
        
        except:
            write_log_message("Thread expansion_listener Interrupt received, stopping...")
            self.emergency_stop()   
                     
'''
-------------------------------------------------------------------------------------
-------------------------------- Movement calculation -------------------------------
-------------------------------------------------------------------------------------
'''
def spatial_observation(self):
    self.calculate_neighbors_distance_sink()
    self.demand_neighbors_info() # return after gathering all info
    self.check_Ownership()

# This called inside the lock so should not contain any lock or it will be dead
def findMinDistances_niegboor(self):
    min_distance = min(self.neighbor_list, key=lambda x: x["distance"])["distance"]
    self.min_distance_dicts =[s["name"] for s in self.neighbor_list if s["distance"] == min_distance]

def set_priorities(self):
    self.list_finished_update.wait()
    with self.lock_neighbor_list: # Writing in the list should be locked 
        denom= 4.0 * self.neighbor_list[0]["distance"] # 4*distance of s0 from sink
        findMinDistances_niegboor(self)
        for s in self.neighbor_list:
            if  int(s["name"][1:]) in self.allowed_spots:
                if s["drones_in"] == 0 and denom !=0: # free spot
                    s["priority"]= s["distance"]* C /denom
                else: # s is occupied
                    if s["name"] in self.min_distance_dicts: #close to sink
                        s["priority"]= float("inf")
                    else: # random float between [w*c+eps, w+1*c[
                        s["priority"]= random.uniform(s["drones_in"]* C + eps, (s["drones_in"]+1)*C)
            else:
                s["priority"]= float("inf")
        if self.allowed_spots: # This constraint should be used only one time after the balancing to avoid going behind the border again
            self.allowed_spots=[]

def find_priority(self):
    min_Priority = min(self.get_neighbor_list(), key=lambda x: x["priority"])["priority"]
    spot_to_go =[s["name"] for s in self.get_neighbor_list() if s["priority"] == min_Priority]
    return int(spot_to_go[0][1:])

def neighbors_election(self):
    # Only free whil be consider in the election 
    id_free = [id for id, state in zip(self.get_current_spot()["drones_in_id"], self.get_current_spot()["states"]) if state == Free]
    return min(id_free) # return the min id of a drone is in state Free

def sink_movement_command(self,vehicle,drones_id):
    assigned_spots=assign_spots(drones_id) 
    if assigned_spots: 
        for ids, spot in assigned_spots.items():
            angle, distance = self.convert_spot_angle_distance(spot)
            if check_gps_fix(vehicle): # GPS data are correct
                current_lat = vehicle.location.global_relative_frame.lat
                current_lon = vehicle.location.global_relative_frame.lon
                long, lat= new_coordinates(current_lon, current_lat, distance , angle)
                msg= build_movement_command_message(ids,spot, long, lat)
                send_msg(msg)
                # Wait until arrival, id*spacing + self.ref_alt time for take off where the hight depend on the drone ID
                time.sleep(((a/defined_groundspeed)+1)+ (ids*spacing + self.ref_alt)+2) # +2 more time to enusre that the drone arrived 
            else:
                # It is command to drone to start,( 0,0) is null island where it is imposible to start from
                msg= build_movement_command_message(id,spot, 0, 0)

def initial_movement(self,vehicle,ID, spot, lon, lat):
    if ID!=1 and ID==self.id: # drone is not sink it is targeted (skink=1 )
        self.update_location(spot) # update the destination even before arriving ( all drones in sky will know the next drone heading in advance)
        self.take_off_drone(vehicle)
        
        if lon!=0 and lat!=0:
            if check_gps_fix(vehicle) and use_GPS:
                self.move_using_coord(vehicle, lon, lat)
            elif (not check_gps_fix(vehicle))  and (not use_GPS):
              search_for_sink_tag(vehicle)
              self.move_to_spot(vehicle, spot)              
            else: # GPS fixed not fixed with use gps set true 
                self.emergency_stop()
        else:
            if not use_GPS:
                #use image to fly on the top of sink
                search_for_sink_tag(vehicle)
                self.move_to_spot(vehicle, spot)
            else:
                self.emergency_stop()
        
        if self.id==2 and Xbee_change_range: # only first drone does the range calibration if this option is activated in operational_Data
            msg= build_calibration_message(1,0)
            send_msg(msg)

def calibration_ping_pong(self, vehicle, msg ):
    indicator, xbee_range= decode_calibration_message(msg)
    if indicator==1 and xbee_range==0: # still calibrating
        if self.id==1:# only the sink respond
            respond_msg= build_calibration_message(1,0)
            send_msg( respond_msg)

        elif self.id==2: # Second drone 
            while True:
                msg= build_calibration_message(1,0)
                send_msg(msg)
                time.sleep(1)
                rec_messge= self.retrieve_msg_from_buffer()
                if rec_messge != None: # A message recived
                    if msg.startswith(Calibration.encode()) and msg.endswith(b'\n'):
                        indicator, xbee_range= decode_calibration_message(rec_messge)
                        if indicator==1 and xbee_range==0:
                            set_to_move(vehicle)
                            # move on the same angle with 1m
                            move_body_PID(vehicle,angle, 1)
                            hover(vehicle)
                            set_a(a+1)# increae a by 1
                else: # No message avilable
                    # Move to the opposit direction
                    angle= normalize_angle(angle -180) # Calculate the opposit angle
                    set_to_move(vehicle)
                    # move on the same angle with 1m
                    move_body_PID(vehicle,angle, 1)
                    hover(vehicle)
                    set_a(a-1)
                    break
            msg= build_calibration_message(-1,a)
    elif indicator==-1 and xbee_range>0:
        if self.id==1:
            respond_msg= build_calibration_message(0,xbee_range)
            self.send_msg(respond_msg)
    elif indicator==0 and xbee_range>0:
        self.update_xbee_range(a)
        set_a(a)
        clear_buffer()


'''
-------------------------------------------------------------------------------------
--------------------------Identify drones by the sink only---------------------------
-------------------------------------------------------------------------------------
'''
def initialize_collect_drones_info_timer(self):
    reset_collect_drones_info_timer(self)
    while True:
        with self.collect_drones_info_timer_lock:
            self.remaining_collect_time -= 0.1
            if self.remaining_collect_time <= 0:
                    break
        time.sleep(0.1)

def reset_collect_drones_info_timer(self):
    with self.collect_drones_info_timer_lock:
        self.remaining_collect_time=60 # wait one minute 

def update_initial_drones_around(self,msg):
    # This function will be called by the listener thread 
    # No need for lock to update collected_ids becauset this list will be used by main thread only after the timer is up  
    found_id= decode_identification_message(msg)
    if (found_id not in self.collected_ids) and (self.remaining_collect_time>=0):
        self.collected_ids.append(found_id)
        msg=build_identification_message(Identification_Caught_header, found_id)
        send_msg(msg)
        reset_collect_drones_info_timer(self)

def sync_Identification(self):
    '''
    Each drone that is not the sink will keep sending its ID until receiving a 
    confirmation from the sink that the identification is received
    '''
    while(not self.sink_handshake.is_set()):
        msg=build_identification_message(Identification_header, self.id)
        send_msg(msg)
        time.sleep(2)

def assign_spots(drones_id):
    # Use round robin to assign a spot to each drone to maintain good equal distribution as possible
    spots = [1,2, 3, 4, 5, 6]
    assignments = {}
    spot_index = 0

    while drones_id:
        num = drones_id.pop(0)  # Remove the first number from the list
        spot = spots[spot_index % len(spots)]  # Assign it to a spot
        assignments[num] = spot
        spot_index += 1

    return assignments

'''
-------------------------------------------------------------------------------------
---------------------------------Mange allowed spots---------------------------------
-------------------------------------------------------------------------------------
'''
def save_unoccupied_spots_around_border(self):
    # Save the spots they are unoccupied to dont back behind border in the next expansion
    if self.get_state()==Border or self.get_state()==Irremovable_boarder:
        self.allowed_spots = [int(neighbor['name'][1:]) for neighbor in self.get_neighbor_list() if neighbor['drones_in'] == 0]


def reset_allowed_spots(self):
    # This only done only after leaving the border not before 
    # very possible to have election beween all dronens one the border so if this delted the drone will back 
    # This constraint should be used only one time after the balancing to avoid going behind the border again
    if (not self.get_previous_state()==Border) or (not self.get_previous_state()==Irremovable_boarder):
        self.allowed_spots =[0,1,2,3,4,5,6]

'''
-------------------------------------------------------------------------------------
----------------------------------- Main functions ----------------------------------
-------------------------------------------------------------------------------------
'''

def expand_and_form_border(self,vehicle):
    
    self.elected_droen_arrived= threading.Event()
    if self.id==1:
        self.update_location(0)
        self.change_state_to(Owner)

    spatial_observation(self)
    while self.get_state() !=Owner:
        set_priorities(self)
        self.destination_spot= find_priority(self)
        self.elected_id= neighbors_election(self)
        
        if self.elected_id== self.id: # current drone is elected one to move
            if self.destination_spot != 0: # Movement to another spot not staying 
                write_log_message (f" elected to go to S {self.destination_spot}")
                self.move_to_spot(vehicle, self.destination_spot)
                # After move_to_spot retuen it means arrivale 
                movement_done_msg= build_expan_elected(self.id)
                send_msg(movement_done_msg)
                
                if self.movemnt_from_border==False:
                   reset_allowed_spots(self)
                   self.movemnt_from_border=True
        else:
           # Wait untile the elected drone to arrive to next spot.
           self.elected_droen_arrived.wait() 
           self.elected_droen_arrived.clear()
           self.rearrange_neighbor_statically_upon_elected_arrival (self.elected_id, self.destination_spot)  


        write_log_message("checking for update the state")
        spatial_observation(self)

    write_log_message("Drone is owner")
    send_msg(self.build_spot_info_message(Response_header))
    self.demand_neighbors_info()
    
    write_log_message("Start border formation")
    Form_border(self)
    # Wait until all border messages are processed and the current topology is saved upon forming border to be used for border verification 
    time.sleep(exchange_data_latency)  
    clear_buffer()

    write_log_message("Verify the border formation")
    if self.border_formed != False:
        border_well_confirmed= confirm_border_connectivity(self)
        if self.get_current_spot()["drones_in"]==1 and border_well_confirmed:
            write_log_message (" Drone is Alone Go to ref ")
            try: 
                go_to_ref_altitude(vehicle,self.ref_alt)
            except:
                write_log_message("An error occurred while go_to_ref_altitude")
                self.emergency_stop()    
        # If the border is not formed you can add reformation border again 
        # re_form_border(self)
    else:
        write_log_message("Return home border is not formed")
        self.emergency_stop()
           
def first_exapnsion (self, vehicle):
    # Lance a thread to read messages continuously
    xbee_receive_message_thread = threading.Thread(target=expansion_listener, args=(self,vehicle)) #pass the function reference and arguments separately to the Thread constructor.
    xbee_receive_message_thread.start()
    self.start_expanding= threading.Event()
    self.elected_droen_arrived= threading.Event()
    # First movement started by commands of the sink
    if self.id==1: # Sink:
        write_log_message("Sink collect data")
        initialize_collect_drones_info_timer(self) # Sink waiting for the drones to make themselves known befor start
        self.take_off_drone(vehicle)
        sink_movement_command(self,vehicle,self.collected_ids)
        # The end send message referes that all in position
        msg= build_movement_command_message(-1,-1, 0, 0)
        send_msg(msg)
    else:
        write_log_message("Send ID and wait for command")
        sync_Identification(self)
        self.start_expanding.wait()
        self.start_expanding.clear()
    
    write_log_message("The first movement is done by all drones, start expansion")
    expand_and_form_border(self, vehicle)
    
    self.expansion_stop.set()
    xbee_receive_message_thread.join() # stop listening to message
    self.expansion_stop.clear()
    clear_buffer()

    save_unoccupied_spots_around_border(self)
    # Time guarantees that all drones begin the searching procedure simultaneously and synchronized.
    time.sleep(sync_time)
    self.search_for_target() # This is blocking until the end of movement
    self.elected_id=None 
    # Since broadcast messages might still be circulating while retrieval has stopped, there could be leftover messages in the buffer.
    # It's essential to clear the buffer before the next phase to prevent any surplus.
    
def further_expansion (self,vehicle):
    # Lance a thread to read messages continuously
    xbee_receive_message_thread = threading.Thread(target=expansion_listener, args=(self,vehicle)) #pass the function reference and arguments separately to the Thread constructor.
    xbee_receive_message_thread.start()
    
    if self.get_state() == Border:
        self.change_state_to(Owner) # The border save it is own spot
    elif self.get_state() == Irremovable_boarder:
        self.change_state_to(Irremovable)
    else: # State is Free the only free will move 
        self.movemnt_from_border=False
    
    # All drones should follow same process (sync between all drones)
    expand_and_form_border(self,vehicle)
    self.expansion_stop.set()
    xbee_receive_message_thread.join() # stop listening to message
    self.expansion_stop.clear()
    clear_buffer()
    
    save_unoccupied_spots_around_border(self)
    # Time guarantees that all drones begin the searching procedure simultaneously and synchronized.
    time.sleep(sync_time)
    self.search_for_target() # This is blocking until the end of movement
    self.elected_id=None 
    