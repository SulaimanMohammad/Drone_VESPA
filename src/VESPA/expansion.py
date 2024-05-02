from .VESPA_module import *
from .form_border_one_direction import *
# from .form_border_tow_direction import *

set_env(globals())

'''
-------------------------------------------------------------------------------------
---------------------------------- Communication ------------------------------------
-------------------------------------------------------------------------------------
'''
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
    #if not self.Forming_Border_Broadcast_REC.is_set():
    if (not self.expansion_stop.is_set()) and (not self.Emergency_stop.is_set()):
        return True
    else: 
        return False

def retrieve_messages_process(queue, shared_object):
    """
    Function to be run by the process. It retrieves messages and puts them in the queue.
    """
    while not shared_object.is_set():
        msg = retrieve_msg_from_buffer(shared_object)
        queue.put(msg)  # Put the message into the queue


def expansion_listener (self,vehicle):
    # Create a Queue for process-thread communication
    # msg_queue = Queue()

    # # Create and start the process
    # p = Process(target=retrieve_messages_process, args=(msg_queue, self.Forming_Border_Broadcast_REC))
    # p.start()

    while check_continuity_of_listening(self):
        # time.sleep(0.1)
        #self.manage_xbee_while_movement()        
        msg= retrieve_msg_from_buffer(self.expansion_stop)
        #if not msg_queue.empty():
        #    msg = msg_queue.get()  # Retrieve a message from the queue        
        #print("msg rec start with ",msg)
        if msg.startswith(Emergecy_header.encode()) and msg.endswith(b'\n'):
            # brodcast it again
            emergency_msg= self.build_emergency_message()
            send_msg(emergency_msg)
            print("retuen home")
            self.Emergency_stop.set()
            self.return_home(vehicle)
            break
            # need to send preodically 

        self.exchange_neighbors_info_communication(msg)
        if msg.startswith(Movement_command.encode()) and msg.endswith(b'\n'):
            id, spot, lon, lat= decode_movement_command_message(msg)
            if id==-1 and spot==-1 and lon==0 and lat==0: # mean all drone are in sky
                self.start_expanding.set()
            else:
                initial_movement(self, vehicle,id, spot, lon, lat)

        # elif msg.startswith(Calibration) and msg.endswith("\n"):
        #     calibration_ping_pong(self, vehicle, msg )

        elif msg.startswith(Expan_header.encode()) and msg.endswith(b'\n'):
            handel_elected_drone_arrivale(self, msg)

        # elif msg.startswith(Arrival_header.encode()) and msg.endswith(b'\n'):
        #     handel_broken_into_spot(self, msg)

        # elif msg.startswith(Inherit_header.encode()) and msg.endswith(b'\n'):
        #     handel_inheritence_message(self, msg)
            
        elif msg.startswith(Forming_border_header.encode()) and msg.endswith(b'\n'): # message starts with F end with \n
            #pass
            # while self.in_movement.is_set():
            #     time.sleep(0.01)
            #if self.state==Owner:  
            form_border_one_direction(self,Forming_border_header,msg)
                #form_border_two_direction(self,Forming_border_header,msg)

        elif msg.startswith(Verify_Border.encode()) and msg.endswith(b'\n'):
            verify_border(self,Verify_Border,msg)
            
    now = datetime.now()
    print(now.strftime("%H:%M:%S.%f"),":finished lsitene",)
    #p.join()
                        
'''
-------------------------------------------------------------------------------------
-------------------------------- Movement calculation -------------------------------
-------------------------------------------------------------------------------------
'''
def spatial_observation(self):
    #self.list_finished_update.wait()
    self.calculate_neighbors_distance_sink()
    self.demand_neighbors_info() # return after gathering all info
    #self.list_finished_update.wait()
    #self.correct_states_after_comm()
    self.check_Ownership()

# dont use get_neighbor_list because it uses locker and set_priority that calles it, amready locked 
def findMinDistances_niegboor(self):
    min_distance = min(self.neighbor_list, key=lambda x: x["distance"])["distance"]
    self.min_distance_dicts =[s["name"] for s in self.neighbor_list if s["distance"] == min_distance]

# Write into the list 
def set_priorities(self):
    self.list_finished_update.wait()
    with self.lock_neighbor_list:
        denom= 4.0 * self.neighbor_list[0]["distance"] # 4*distance of s0 from sink
        # findMinDistances_niegboor(self)
        min_distance = min(self.neighbor_list, key=lambda x: x["distance"])["distance"]
        self.min_distance_dicts =[s["name"] for s in self.neighbor_list if s["distance"] == min_distance]
        
        for s in self.neighbor_list:
            if int(s["name"][1:]) in self.allowed_spots:
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
    id_free = [id for id, state in zip(self.get_current_spot()["drones_in_id"],self.get_current_spot()["states"]) if state == Free] #only free whill be consider in the election 
    return min(id_free) # return the min id of a drone is in state Free

def sink_movement_command(self,vehicle,id):
    #destination_spot_random = int(random.randint(1, 6))
    if id==1:
        destination_spot_random=3
    else:
        destination_spot_random=6

    angle, distance = self.convert_spot_angle_distance(destination_spot_random)
    if check_gps_fix(vehicle): # GPS data are correct
        current_lat = vehicle.location.global_relative_frame.lat
        current_lon = vehicle.location.global_relative_frame.lon
        long, lat= new_coordinates(current_lon, current_lat, distance , angle)
        print(long, lat)
        msg= build_movement_command_message(id,destination_spot_random, long, lat)
        print("sink_movement_command",msg )
        send_msg(msg)
        time.sleep(((a/defined_groundspeed)+1)+ (id*spacing + self.ref_alt))# wait until arrival
    else:
        # It is command to drone to start,( 0,0) is null island where it is imposible to start from
        msg= build_movement_command_message(id,destination_spot_random, 0, 0)

def initial_movement(self,vehicle,id, spot, lon, lat):
    if id !=0 and id==self.id: # drone is not sink and it is targeted
        self.update_location(spot) # update the destination even before arriving ( better for commuiniaation)
        arm_and_takeoff(vehicle,self.drone_alt)
        if lon!=0 and lat!=0:
            time.sleep(2)
            point1 = LocationGlobalRelative(lat,lon ,self.drone_alt)
            vehicle.simple_goto( point1, groundspeed=defined_groundspeed)
            # simple_goto will retuen after the command is sent, thus you need to sleep to give the drone time to move
            time.sleep((a/defined_groundspeed)+1 )
            print("arrived")
            #vehicle.mode    = VehicleMode("LOITER") #loiter mode and hover in your place
            time.sleep(1)
            vehicle.mode     = VehicleMode("GUIDED")


        else:
            #use image to fly on the top of sink
            search_for_sink_tag(vehicle)
            self.move_to_spot(vehicle, spot)
        # #loiter mode and hover in your place if it is before sleep then the drone will not move
        # angle, distance = self.convert_spot_angle_distance(spot)
        # set_yaw_to_dir_PID( vehicle, angle) # set the angle in the same direction taken since simple goto can include rotation
        # if self.id==1: # only first drone does the range calibration
        #     msg= build_calibration_message(1,0)
        #     send_msg(msg)

def calibration_ping_pong(self, vehicle, msg ):
    indicator, xbee_range= decode_calibration_message(msg)
    if indicator==1 and xbee_range==0: # still calibrating
        if self.id==0:# only the sink respond
            respond_msg= build_calibration_message(1,0)
            send_msg( respond_msg)

        elif self.id==1:
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
        if self.id==0:
            respond_msg= build_calibration_message(0,xbee_range)
            self.send_msg(respond_msg)
    elif indicator==0 and xbee_range>0:
        self.update_xbee_range(a)
        set_a(a)
        clear_buffer()



'''
-------------------------------------------------------------------------------------
--------------------------------- Forming the border---------------------------------
-------------------------------------------------------------------------------------
'''


def save_unoccupied_spots_around_border(self):
    # save spots number that doesnt contains any drone from the point of border 
    # self.allowed_spots = [neighbor['name'] for neighbor in  self.neighbor_list if neighbor['drones_in'] == 0]
    self.allowed_spots = [int(neighbor['name'][1:]) for neighbor in self.get_neighbor_list() if neighbor['drones_in'] == 0]

'''
-------------------------------------------------------------------------------------
----------------------------------- Main functions ----------------------------------
-------------------------------------------------------------------------------------
'''
def expand_and_form_border_try(self, vehicle):
    # xbee_receive_message_thread = threading.Thread(target=expansion_listener, args=(self,vehicle,)) #pass the function reference and arguments separately to the Thread constructor.
    # xbee_receive_message_thread.start()
    # xbee_receive_message_thread = Process(target=expansion_listener, args=(self,))
    # xbee_receive_message_thread.start()
    # xbee_receive_message_process.join() # Optionally wait for the process to finish
    self.elected_droen_arrived= threading.Event()
    if self.id==0:
        self.update_location(0)
        self.change_state_to(Owner)
        # for station in self.neighbor_list:
        #     if station['drones_in'] > 0:
        #         print(station)
        #self.Forming_Border_Broadcast_REC.wait()
        #time.sleep(10)
    # else:
    #     destination_spot_random = 2
    #     print("before movement distance is:", self.spot["distance"])
    #     print ("go to S", destination_spot_random)
    #     self.move_to_spot(vehicle, destination_spot_random)
    #     print("after move_to_spot distance is:", self.spot["distance"])
    #     #self.update_location(destination_spot_random)
    #     print("after update_location distance is:", self.spot["distance"])

    
    print("spot", self.spot)
    spatial_observation(self)
    print("After")
    for station in self.get_neighbor_list():
        if station['drones_in'] > 0:
            print(station)
    print(self.state)
    print(self.spot )        
    print(self.get_neighbor_list() )        
    while self.get_state() !=Owner:
        set_priorities(self)
        print("set_priorities")
        self.destination_spot= find_priority(self)
        self.elected_id= neighbors_election(self)
        
        if self.elected_id== self.id: # current drone is elected one to move
            print("elected", self.elected_id)
            if self.destination_spot != 0: # Movement to another spot not staying 
                print ("move to S", self.destination_spot)
                self.move_to_spot(vehicle, self.destination_spot)
                #self.update_location(self.destination_spot)
                #time.sleep(2)
                print("arrived from main")
                # After move_to_spot retuen it means arrivale 
                movement_done_msg= build_expan_elected(self.id)
                send_msg(movement_done_msg)
        else:
           print( "waiting for flag")
           # Wait untile the elected drone to arrive to next spot.
           self.elected_droen_arrived.wait() 
           self.elected_droen_arrived.clear()
           self.rearrange_neighbor_statically_upon_elected_arrival (self.elected_id, self.destination_spot)  
           print( "after election")
           for station in self.get_neighbor_list():
                if station['drones_in'] > 0:
                    print(station)

        print("checking for update the state")
        spatial_observation(self)
        print("in loop")
        for station in self.get_neighbor_list():
            if station['drones_in'] > 0:
                print(station)
    
    send_msg(self.build_spot_info_message(Response_header))

    self.demand_neighbors_info()       
    # check also that no electin message around                                         ):
    while (self.get_current_spot()['drones_in']>1) or (not(all(neighbor['drones_in'] in [0, 1] for neighbor in self.get_neighbor_list()))) or not self.all_neighbor_spots_owned():

        # Before initiating the border procedure, it's important to wait for some time to ensures that the drone is alone in its spot.
        # This step eliminates the possibility of erroneously considering a drone as a border-candidate when another drone in the same spot is about to move.
        '''
        The loop continues as long as any of these conditions are true:

        The current spot has more than one drone: so it is not alone and one of the drone will populate one of the neigbor spot so ait for that to consider border cndidiate 
        Any neighbor has more than one drone: oso the neigbor has many and some will move and occupy a spot around wait this before check the border  
        All neighbors have at least one drone
        '''
        time.sleep(1)
        print(" waiting to check for border")
        self.demand_neighbors_info()
        print( " finished observing")
        for station in self.get_neighbor_list():
            if station['drones_in'] > 0:
                print(station)
        
    print("forming border")
    spatial_observation(self)
    for station in self.get_neighbor_list():
        if station['drones_in'] > 0:
            print(station)

    if self.spot["drones_in"]==1:
        print (" Drone is Alone ")
        print ( "Go to ref", self.ref_alt)

    Forme_border(self)
    print(" after finished border drone is ", self.state)
    clear_buffer()
    self.demand_neighbors_info() # needed to update what neigbor become border 
    for station in self.get_neighbor_list():
        if station['drones_in'] > 0:
            print(station)
    time.sleep(5)
    print(" VERFIFY ")
    if self.border_formed != False:
        confirm_border_connection(self)
    else:
        emergency_msg= self.build_emergency_message()
        send_msg(emergency_msg)
        print("retuen home")
        self.return_home(vehicle)

    self.expansion_stop.set()
    # xbee_receive_message_thread.join() # stop listening to message    
    self.expansion_stop.clear()

    time.sleep(5)
    alive_threads = threading.enumerate()
    for t in alive_threads:
        print(f"Thread Name: {t.name}. Alive: {t.is_alive()}")

    print( "x", self.positionX , "y",self.positionY)
    print("end ")

def expand_and_form_border(self,vehicle):
    spatial_observation(self) 
    while self.get_state() !=Owner:
        set_priorities(self)
        destination_spot= find_priority(self)
        self.elected_id= neighbors_election(self)
        if self.elected_id== self.id: # current drone is elected one to move
            if destination_spot != 0: # Movement to another spot not staying 
                print ("go to S", destination_spot)
                self.move_to_spot(vehicle, destination_spot)
                # After move_to_spot retuen it means arrivale 
                self.manage_xbee_while_movement()
                # that means upon each movement there will be message sent , even if there are 2 drones on free and one owner then the one moves will send the message 
                movement_done_msg= build_expan_elected(self.id)
                send_msg(movement_done_msg)
        else:
           # Wait untile the elected drone to arrive to next spot.
           self.elected_droen_arrived.wait() 
           self.elected_droen_arrived.clear()
           self.rearrange_neighbor_statically_upon_elected_arrival (self.elected_id, destination_spot)

        print("checking for update the state")
        spatial_observation(self)
        
    while not (all(neighbor['drones_in'] in [0, 1] for neighbor in self.neighbor_list) or
               all(neighbor['drones_in'] > 0 for neighbor in self.neighbor_list)):
        # Before initiating the border procedure, it's important to wait for some time to ensures that the drone is alone in its spot.
        # This step eliminates the possibility of erroneously considering a drone as a border-candidate when another drone in the same spot is about to move.
        time.sleep(sync_time)
        self.demand_neighbors_info()

    Forme_border(self)# will not return until the drones receive boradcast of forming border
   
    # Save the spots they are unoccupied to dont back behind border in the next expansion
    if self.get_state()==Border:
        save_unoccupied_spots_around_border(self)     

    # Time guarantees that all drones begin the searching procedure simultaneously and synchronized.
    time.sleep(sync_time)
    self.search_for_target() # This is blocking until the end of movement

    # Reset variables for the next iteration
    self.rec_propagation_indicator=[] 
    self.rec_candidate=[]
    self.direction_taken=[] 
    self.elected_id=None     
    
def first_exapnsion (self, vehicle):
    # Lance a thread to read messages continuously
    xbee_receive_message_thread = threading.Thread(target=expansion_listener, args=(self,vehicle,)) #pass the function reference and arguments separately to the Thread constructor.
    xbee_receive_message_thread.start()
    self.start_expanding= threading.Event()
    self.elected_droen_arrived= threading.Event()
    # First movement started by commands of the sink
    if self.id==0: #sink:
        arm_and_takeoff(vehicle,self.drone_alt)
        time.sleep(2)
        # with open('Operational_Data.txt', 'r') as file:
        #     for line in file:
        #         # Check if line contains max_acceleration
        #         if "drones_number" in line:
        #             drones_number = float(line.split('=')[1].strip())
        #     print(drones_number )
        drones_number=2
        for i in range(1,drones_number+1):
            sink_movement_command(self,vehicle,i)
        # The end send message referes that all in position
        msg= build_movement_command_message(-1,-1, 0, 0)
        send_msg(msg)
    else:
        self.start_expanding.wait()
        self.start_expanding.clear()
    # expand_and_form_border(self, vehicle)
    expand_and_form_border_try(self, vehicle)
    
    # Since broadcast messages might still be circulating while retrieval has stopped, there could be leftover messages in the buffer.
    # It's essential to clear the buffer before the next phase to prevent any surplus.
    clear_buffer()
    xbee_receive_message_thread.join() # stop listening to message
    
def further_expansion (self,vehicle):
    # Lance a thread to read messages continuously
    xbee_receive_message_thread = threading.Thread(target=expansion_listener, args=(self,)) #pass the function reference and arguments separately to the Thread constructor.
    xbee_receive_message_thread.start()
    
    if self.get_state() == Border:
        self.change_state_to(Owner) # The border save it is own spot
    elif self.get_state() == Irremovable_boarder:
        self.change_state_to(Irremovable)
    else: # State is Free
        expand_and_form_border(self,vehicle)
    
    clear_buffer()
    xbee_receive_message_thread.join() 
    

