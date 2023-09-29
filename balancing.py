from expan import *

def communication_balancing(self):
    msg= self.retrive_msg_from_buffer() 
    # Receiving message asking data 
    if msg == Demand_header.encode() + b'\n':
        data_msg= self.build_spot_info_message(Reponse_header) # Build message that contains all data
        self.send_msg(data_msg)

    # Receiving message containing data     
    if msg.startswith(Reponse_header.encode()) and msg.endswith(b'\n'):
        self.neighbors_list_updated = threading.Event()
        positionX, positionY, state, id_value= self.decode_spot_info_message(msg)
        self.update_neighbors_list(positionX, positionY, state, id_value )
        self.neighbors_list_updated.set()


def search_going_to_border(self):
    border_found=False
    
    #Find states with 'border'
    border_states = [s for s in self.neighbor_list if 'border' in s['states']]
    
    # If only one border state is found
    if len(border_states) == 1:
        border_found= True
        return border_found, int(border_states[0]['name'][1:])
    
    # If multiple border states are found
    elif len(border_states) > 1:
        # Sort by drones_in
        border_found= True
        border_states.sort(key=lambda x: x['drones_in'])
        
        # Filter states with the least drones
        least_drones = border_states[0]['drones_in']
        least_drones_states = [s for s in border_states if s['drones_in'] == least_drones]
        
        # Choose randomly the states with the least drones if many have same numbers of drones in 
        chosen_spot= random.choice(least_drones_states)
        return border_found, int(chosen_spot['name'][1:])
    
    # If no border state is found, find the spot with the maximum distance (close to the border)
    else:
        max_distance_spot = max(self.neighbor_list, key=lambda x: x['distance'])
        return border_found, int(max_distance_spot['name'][1:]) 


# the border is the chef of the spot 
# end when a message from each border to another that balancig is done 

# This function saves the spots that are occupied regarless if it is in the dominated direction or not 
def save_occupied_spots(self):
    occupied_spots = [int(spot["name"][1:]) for spot in self.neighbor_list if spot["drones_in"] != 0]
    return occupied_spots

def construct_allowed_spots(self, border_neighbors ):
    # this should be done in balancing when the border all of it known
    # save occupied spot sould be only for the free drone or irremovable ( dont consider the border )
    self.border_neighbors = self.save_occupied_spots() # The border drone will save the occupied spot for the next expansion 


def balancing(self, vehicle):
    self.xbee_receive_message_thread = threading.Thread(target=self.communication_balancing, args=(self,)) #pass the function reference and arguments separately to the Thread constructor.
    self.xbee_receive_message_thread.start()
    brder_found= False
    if self.state == Free: 
        while border_found == False : # no border in the nieghbor
            self.build_data_demand_message()
            self.neighbors_list_updated.wait()
            self.neighbors_list_updated.clear()
            border_found, spot_to_go = search_going_to_border(self)
            # Move 
            self.direction_taken.append( spot_to_go)
            angle, distance = self.convert_spot_angle_distance(spot_to_go)
            move_body_PID(vehicle,angle, distance)
            self.update_location(spot_to_go)




