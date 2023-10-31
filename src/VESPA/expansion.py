from VESPA_module import *
'''
-------------------------------------------------------------------------------------
---------------------------------- Communication ------------------------------------
-------------------------------------------------------------------------------------
'''
def build_movement_command_message(self,id, spot, float1, float2):
    # Movement messages are encoded using string beause they are done only once, efficiency accepted
    # Convert numbers to string and encode
    id_str= str(id).encode()
    s_str=str(spot).encode()
    float1_str = str(float1).encode()
    float2_str = str(float2).encode()
    # Construct message
    message = Movement_command.encode() + id_str +b','+s_str+ b','+ float1_str + b',' + float2_str + b'\n'
    return message

def decode_movement_command_message(self,message):
    # Remove header and terminal
    content = message[1:-1]
    # Split by the comma to get floats
    parts = content.split(b',')
    id = int(parts[0])
    s = int(parts[1])
    float1 = float(parts[2])
    float2 = float(parts[3])
    return id, s, float1, float2

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

def build_expan_elected(self,id):
    # Determine max byte count for numbers
    max_byte_count = self.determine_max_byte_size(id)
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
        self.check_border_candidate_eligibility() # use only the updated list and see if the current drone still candidate
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

def handel_elected_drone_arrivale(self,msg):
    rec_id=self.decode_expan_elected(msg)
    if rec_id==self.elected_id:
        self.elected_droen_arrived.set()
    
def receive_message (self,vehicle):
    self.Forming_Border_Broadcast_REC = threading.Event()

    while not self.Forming_Border_Broadcast_REC.is_set():

        msg= self.retrive_msg_from_buffer()

        self.exchange_neighbors_info_communication(msg)

        if msg.startswith(Movement_command) and msg.endswith("\n"):
            id, spot, lon, lat=self.decode_movement_command_message(msg)
            if id==-1 and spot==-1 and lon==0 and lat==0: # mean all drone are in sky
                self.start_expanding.set()
            else:
                self.initial_movement(vehicle,id, spot, lon, lat)

        elif msg.startswith(Calibration) and msg.endswith("\n"):
            self.calibration_ping_pong(vehicle, msg )

        elif msg.startswith(Expan_header.encode()) and msg.endswith(b'\n'):
            self.handel_elected_drone_arrivale(msg)

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
                self.forward_broadcast_message(Forming_border_header,candidate)
                self.Forming_Border_Broadcast_REC.set()


            elif self.id in  target_ids: # the drone respond only if it is targeted
                if candidate == self.id: # the message recived contains the id of the drone means the message came back
                    self.circle_completed ()

                else: # the current drone received a message from a candidate border so it needs to forward it
                    self.find_msg_direction_forward(rec_propagation_indicator,target_ids,sender,candidate )

            else: # Drone is not targeted ( doesnt matter it it is free or candidate) thus it drops the message
                    # Do anything but wait for end of the expansion broadcast
                    continue


'''
-------------------------------------------------------------------------------------
-------------------------------- Movement calculation -------------------------------
-------------------------------------------------------------------------------------
'''

# All float numbers are _.xx 2 decimal
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

def spatial_observation(self):
    self.demand_neighbors_info() # return after gathering all info  
    self.check_Ownership()

def findMinDistances_niegboor(self):
    min_distance = min(self.neighbor_list, key=lambda x: x["distance"])["distance"]
    self.min_distance_dicts =[s["name"] for s in self.neighbor_list if s["distance"] == min_distance]

def set_priorities(self):
    denom= 4.0 * self.neighbor_list[0]["distance"] # 4*distance of s0 from sink
    self.findMinDistances_niegboor()
    for s in self.neighbor_list:
        if  s["name"] not in self.allowed_spots:
            if s["drones_in"] == 0 : # free spot
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
    min_Priority = min(self.neighbor_list, key=lambda x: x["priority"])["priority"]
    spot_to_go =[s["name"] for s in self.neighbor_list if s["priority"] == min_Priority]
    return int(spot_to_go[0][1:])

def neighbors_election(self):
    id_free = [id for id, state in zip(self.spot["drones_in_id"], self.spot["states"]) if state == Free]
    return min(id_free) # return the min id of a drone is in state Free

def sink_movement_command(self,vehicle,id):
    destination_spot_random = int(random.randint(1, 6))
    angle, distance = self.convert_spot_angle_distance(destination_spot_random)
    if check_gps_fix(vehicle): # GPS data are correct
        current_lat = vehicle.location.global_relative_frame.lat
        current_lon = vehicle.location.global_relative_frame.lon
        long, lat= new_coordinates(current_lon, current_lat, distance , angle)
        msg= self.build_movement_command_message(id,destination_spot_random, long, lat)
        self.send_msg(msg)
        time.sleep((a/defined_groundspeed)+1 )# wait until arrival
    else:
        # It is command to drone to start,( 0,0) is null island where it is imposible to start from
        msg= self.build_movement_command_message(id,destination_spot_random, 0, 0)

def initial_movement(self,vehicle,id, spot, lon, lat):
    if id !=0 and id==self.id: # drone is not sink and it is targeted
        arm_and_takeoff(vehicle,self.hight)
        if lon!=0 and lat!=0:
            time.sleep(2)
            point1 = LocationGlobalRelative(lat,lon ,self.hight)
            vehicle.simple_goto( point1, groundspeed=defined_groundspeed)
            # simple_goto will retuen after the command is sent, thus you need to sleep to give the drone time to move
            time.sleep((a/defined_groundspeed)+1 )
            vehicle.mode    = VehicleMode("LOITER") #loiter mode and hover in your place
            time.sleep(1)
            vehicle.mode     = VehicleMode("GUIDED")
            self.direction_taken.append(spot)
        else:
            #use image to fly on the top of sink
            search_for_sink_tag(vehicle)
            self.move_to_spot(vehicle, spot)
        #loiter mode and hover in your place if it is before sleep then the drone will not move
        angle, distance = self.convert_spot_angle_distance(spot)
        set_yaw_to_dir_PID( vehicle, angle) # set the angle in the same direction taken since simple goto can include rotation
        if self.id==1: # only first drone does the range calibration
            msg= self.build_calibration_message(1,0)
            self.send_msg(msg)

def calibration_ping_pong(self, vehicle, msg ):
    indicator, xbee_range= self.decode_calibration_message(msg)
    if indicator==1 and xbee_range==0: # still calibrating
        if self.id==0:# only the sink respond
            respond_msg= self.build_calibration_message(1,0)
            self.send_msg( respond_msg)

        elif self.id==1:
            while True:
                msg= self.build_calibration_message(1,0)
                self.send_msg(msg)
                time.sleep(1)
                rec_messge= self.retrive_msg_from_buffer()
                if rec_messge != None: # A message recived
                    if msg.startswith(Calibration.encode()) and msg.endswith(b'\n'):
                        indicator, xbee_range= self.decode_calibration_message(rec_messge)
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
            msg= self.build_calibration_message(-1,a)
    elif indicator==-1 and xbee_range>0:
        if self.id==0:
            respond_msg= self.build_calibration_message(0,xbee_range)
            self.send_msg(respond_msg)
    elif indicator==0 and xbee_range>0:
        self.update_xbee_range(a)
        set_a(a)
        self.clear_buffer()

'''
-------------------------------------------------------------------------------------
--------------------------------- Forming the border---------------------------------
-------------------------------------------------------------------------------------
'''
def count_element_occurrences(self):
    # Find the maximum element in the direction_taken list
    max_element = 7  # s0 to s6
    # Create a dictionary to store the frequencies, similar to hash map to count occurrences
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

def check_border_candidate_eligibility(self):
    self.border_candidate=False
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

def Fire_border_msg(self, header):
    target_ids= self.creat_target_list(header)
    # At the beginning  propagation_indicator and target_ids are the same in the source of the message
    propagation_indicator= target_ids
    Msg= self.build_border_message(header,propagation_indicator, target_ids, self.id)
    self.send_msg(Msg)

def Forme_border(self):
    self.check_border_candidate_eligibility()
    if self.border_candidate :
        self.update_candidate_spot_info_to_neighbors() # Useful if the drone arrived and filled a spot made others sourounded
        '''launch a message circulation for current candidat'''
        self.Fire_border_msg(Forming_border_header)
    else:
        #means that the drone is sourounded in the expansion direction it can be set as free
        self.change_state_to(Free)

    # wait until the border procesdure is finished
    self.Forming_Border_Broadcast_REC.wait()
    self.Forming_Border_Broadcast_REC.clear() # reset for the next expansion

def save_unoccupied_spots_around_border(self):
    # save spots that doesnt contains any drone from the point of border 
    self.allowed_spots = [neighbor['name'] for neighbor in  self.neighbor_list if neighbor['drones_in'] == 0]

'''
-------------------------------------------------------------------------------------
----------------------------------- Main functions ----------------------------------
-------------------------------------------------------------------------------------
'''
def expand_and_form_border(self,vehicle):
    self.spatial_observation()
    while self.state !=Owner:
        self.set_priorities()
        destination_spot= self.find_priority()
        self.elected_id= self.neighbors_election()
        if self.elected_id== self.id: # current drone is elected one to move
            if destination_spot != 0: # Movement to another spot not staying 
                print ("go to S", destination_spot)
                self.move_to_spot(vehicle, destination_spot)
                # After move_to_spot retuen it means arrivale 
                movement_done_msg= self.build_expan_elected(self.id)
                self.send_msg(movement_done_msg)
        else:
           # Wait untile the elected drone to arrive to next spot.
           self.elected_droen_arrived.wait() 
           self.elected_droen_arrived.clear()

        print("checking for update the state")
        self.spatial_observation()

    while not (all(neighbor['drones_in'] in [0, 1] for neighbor in self.neighbor_list) or
               all(neighbor['drones_in'] > 0 for neighbor in self.neighbor_list)):
        # Before initiating the border procedure, it's important to wait for some time to ensures that the drone is alone in its spot.
        # This step eliminates the possibility of erroneously considering a drone as a border-candidate when another drone in the same spot is about to move.
        time.sleep(sync_time)
        self.demand_neighbors_info()

    self.Forme_border()# will not return until the drones receive boradcast of forming border
        
    # Reset variables for the next iteration
    self.rec_propagation_indicator=[] 
    self.rec_candidate=[]
    self.direction_taken=[] 
    self.elected_id=None     
    self.xbee_receive_message_thread.join() # stop listening to message

    # save the spots they are unoccupied to dont back behind border in the next expansion
    if self.state==Border:
        self.save_unoccupied_spots_around_border() 

    # Time guarantees that all drones begin the searching procedure simultaneously and synchronized.
    time.sleep(sync_time)
    self.search_for_target() # This is blocking until the end of movement

    # Since broadcast messages might still be circulating while retrieval has stopped, there could be leftover messages in the buffer.
    # It's essential to clear the buffer before the next phase to prevent any surplus.
    self.clear_buffer()

def first_exapnsion (self, vehicle):
    # First movement started by commands of the sink
    if self.id==0: #sink:
        arm_and_takeoff(vehicle,self.hight)
        time.sleep(2)
        with open('Operational_Data.txt', 'r') as file:
            for line in file:
                # Check if line contains max_acceleration
                if "drones_number" in line:
                    drones_number = float(line.split('=')[1].strip())
        for i in range(1,drones_number+1):
            self.sink_movement_command(vehicle,i)
        # The end send message referes that all in position
        msg= self.build_movement_command_message(-1,-1, 0, 0)
        self.send_msg(msg)
    else:
        self.start_expanding.wait()
    self.expand_and_form_border(vehicle)

def further_expansion (self,vehicle):
    if self.state == Border:
        self.change_state_to(Owner) # The border save it is own spot
    elif self.state == Irremovable_boarder:
        self.change_state_to(Irremovable)
    else: # State is Free
        self.expand_and_form_border(vehicle)
