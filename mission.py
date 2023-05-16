from  expan import *
from done_ardupilot import *

vehicle = connect(parse_connect(), wait_ready=False)
set_home_to_zero(vehicle)
arm_and_takeoff(vehicle, 10)

drone= Drone(0.0,0.0) # drone at the sink 
# move randomaly far from the sink
random_dir = int(random.randint(1, 6))
print ("go to S", random_dir)
x,y= drone.direction(random_dir)
move_to(vehicle,x,y)
calculate_relative_pos(vehicle)
drone.update_location(x,y)
# in the new position find the distance of the neigboors 
drone.calculate_neigboors_dis()
drone.check_drones_in_neigboors()
drone.setPriorities()

# be carful it should not be move , it is set the psoition 
# bcause move will use only the direction value as a movement from the current location 

spot= drone.findPriority()
print ("go to S", spot)
x,y = drone.direction(spot)
move_to(vehicle,x,y)
calculate_relative_pos(vehicle)
drone.update_location(x,y)


print ("Coming Home")
vehicle.mode = VehicleMode ("RTL")
time.sleep(10) 

vehicle.close()
