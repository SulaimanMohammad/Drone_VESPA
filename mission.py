from  expan import *
from drone_ardupilot import *

create_log_file(os.path.dirname(os.path.abspath(__file__)),  os.path.splitext(os.path.basename(__file__))[0]) 


vehicle = connect(parse_connect(), wait_ready=False)
set_home_to_zero(vehicle)
arm_and_takeoff(vehicle, 10)

drone= Drone(0.0,0.0) # drone at the sink 
# move randomaly far from the sink
print("current_lat =", vehicle.location.global_relative_frame.lat) 
print("current_lon=", vehicle.location.global_relative_frame.lon)
random_dir = int(random.randint(1, 6)) # 0 not include because it should not be cin the sink
print ("go to S", random_dir)
x,y= drone.direction(random_dir)
#move_to(vehicle,x,y)
calculate_relative_pos(vehicle)
drone.update_location(x,y)

# do not do the calculation until the vehicle arrive to destination
while drone.state !="Alone":
   
    # in the new position find the distance of the neigboors 
    drone.calculate_neigboors_dis()
    print("checking for movement")
    drone.check_drones_in_neigboors()
    drone.setPriorities()

    # be carful it should not be move , it is set the psoition 
    # bcause move will use only the direction value as a movement from the current location 

    spot= drone.findPriority()
    print ("go to S", spot)
    x,y = drone.direction(spot)
    #move_to(vehicle,x,y)
    calculate_relative_pos(vehicle)
    drone.update_location(x,y)
    print("checking for update the state")
    drone.is_it_alone()

drone.search_for_target() # find if there is target in the area or not 
drone.update_state() # it inclueds forming the border
print ("Coming Home")
vehicle.mode = VehicleMode ("RTL")
time.sleep(10) 

vehicle.close()
