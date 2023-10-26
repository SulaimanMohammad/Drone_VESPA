from expan import * 

'''
-------------------------------------------------------------------------------------
---------------------------------- Communication ------------------------------------
-------------------------------------------------------------------------------------
'''

# Movement message are encoded using string beause they are done only once
# Thus it will not problem for efficiency 
def build_movement_command_message(self,id, s, float1, float2):
    # Convert numbers to string and encode
    id_str= str(id).encode()
    s_str=str(s).encode()
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
    num1_encoded = struct.pack('b', id)  # signed byte
    num2_encoded = struct.pack('B', xbee_range)  # unsigned byte
    # Construct message
    message = Calibration.encode + num1_encoded + num2_encoded + b'\n'
    return message

def decode_calibration_message(message):
    # Check for valid header and terminator
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
        self.update_neighbors_list(positionX, positionY, state, id_rec) # No need to mutex since the drone is in border_candidate only in it was Owner and reserved spot
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

def receive_message (self,vehicle):
    self.Forming_Border_Broadcast_REC = threading.Event()
    
    while not self.Forming_Border_Broadcast_REC.is_set():
                    
        msg= self.retrive_msg_from_buffer() 
        time.slee(0.01) # wait to have msgs in the buffer
        
        if msg.startswith(Movement_command) and msg.endswith("\n"):
            id, spot, lon, lat=self.decode_movement_command_message(msg)
            if id==-1 and spot==-1 and lon==0 and lat==0: # mean all drone are in sky 
                self.start_expanding.set()
            else: 
                self.initial_movement(vehicle,id, spot, lon, lat)

        elif msg.startswith(Calibration) and msg.endswith("\n"):
            self.calibration_ping_pong(vehicle, msg ) 

        elif msg.startswith(Expan_header.encode()) and msg.endswith(b'\n'):
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
            
        else: 
            print(" Undefined header or empty buffer")

    

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
    
    if self.allowed_spots: #This constraint should be used only one time after the balancing to avoid going behind the border again
        self.allowed_spots=[]

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
    move_body_PID(vehicle,angle, distance)
    self.update_location(destination_spot)
    self.clear_buffer() # need to clear the bufer from message received on the road 
    # Arrive to steady state and hover then start observing the location
    hover(vehicle)

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
                    set_a(a-1)# increae a by 1
                    break
            msg= self.build_calibration_message(-1,a)
    elif indicator==-1 and xbee_range>0:
        if self.id==0:
            respond_msg= self.build_calibration_message(0,xbee_range)
            self.send_msg(respond_msg)
    elif indicator==0 and xbee_range>0:
        set_a( xbee_range)
        self.clear_buffer()

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

def check_border_candidate_eligibility(self, observe=True):
    self.border_candidate=False  
    # Collect the info about the drones around , and that can be ignored in case of drone arriver after election
    if observe:
        self.check_num_drones_in_neigbors()

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

def Forme_border(self, vehicle):
    # since border_neighbors is used only for border drone then no need for it 
    #self.border_neighbors=[] #erase border_neighbors because no need for it 
    self.check_border_candidate_eligibility()
    
    # Ensure the drone is the only one in its spot, even if this check is done implicitly before in waiting time.

    while self.spot["drones_in"] > 1:
        time.sleep(movement_time)
        self.check_num_drones_in_neigbors()
    
    if self.border_candidate :
        self.update_candidate_spot_info_to_neighbors() # Useful if the drone arrived and filled a spot made others sourounded 
        '''launch a message circulation for current candidat'''
        self.Fire_border_msg(Forming_border_header)

    else: 
        #means that the drone is sourounded in the expansion direction it can be set as free 
        self.change_state_to(Free)
        
    # wait until the border procesdure is finished 
    self.Forming_Border_Broadcast_REC.wait()
    set_to_move(vehicle)
    self.Forming_Border_Broadcast_REC.clear() # reset for the next expansion 

# This function saves the spots that are occupied for the next expansion
def save_occupied_spots(self):
    # save occupied spot sould be only for the free drone or irremovable, The border drone will save the occupied spot for the next expansion 
    self.allowed_spots=[]
    for s in self.neighbor_list:
        # Check if 'free' or 'irremovable' is in the states  except's0'
        if (s['name'] != 's0') and (Free in s['states'] or Irremovable in s['states']):
            self.allowed_spots.append(s['name'])

def search_for_target(self): # find if there is target in the area or not 
    # move in the place and couver it to check if there is target or not 
    pass


'''
-------------------------------------------------------------------------------------
----------------------------------- Main functions ----------------------------------
-------------------------------------------------------------------------------------
'''
def expan_border_search(self,vehicle):
    self.check_Ownership()
    
    while self.state !=Owner:
        
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
                self.move_to_spot(vehicle, destination_spot)
        else: 
            time.sleep(movement_time) # Wait untile the elected drone to leave to next stop.
            continue # do all the steps again escape update location because no movement done yet 
            '''Go to another iteration to redo all process because there is possibility that the spots around have changed'''

        calculate_relative_pos(vehicle)
        print("checking for update the state")
        self.check_Ownership()
    
    # Before initiating the border procedure, it's important to wait for some time to ensures that the drone is alone in its spot.
    # This step eliminates the possibility of erroneously considering a drone as a border-candidate when another drone in the same spot is about to move.
    time.sleep(sync_time)
    

    self.Forme_border(vehicle)# will not return until the drones receive boradcast of forming border

    self.rec_propagation_indicator=[] # rest this indecator for the next iteration 
    self.rec_candidate=[]
    self.direction_taken=[] #rest this taken path for the next iteration
    self.xbee_receive_message_thread.join() # stop listening to message
    
    # save the spots they are occupied to dont back to them in the next expansion when they are released
    if self.state==Border:
        self.save_occupied_spots() 

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
        for i in range(1,drones_number):
            self.sink_movement_command(vehicle,i)
        # The end send message referes that all in position 
        msg= self.build_movement_command_message(-1,-1, 0, 0)
        self.send_msg(msg)
    else: 
        self.start_expanding.wait()
    self.expan_border_search(vehicle)
        
def further_expansion (self,vehicle):
    if self.state == Border:
        self.change_state_to(Owner) # The border save it is own spot 
    elif self.state == Irremovable_boarder:
        self.change_state_to(Irremovable)
    else: # State is Free
        self.expan_border_search(vehicle)
# old border can be free by using the saved occupied spot in it to see if it i still border ot not