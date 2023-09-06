from  expan import *
from drone_ardupilot import *

create_log_file(os.path.dirname(os.path.abspath(__file__)),  os.path.splitext(os.path.basename(__file__))[0]) 




drone= Drone(0.0,0.0) # drone at the sink 
# move randomaly far from the sink
random_dir = int(random.randint(1, 6)) # 0 not include because it should not be cin the sink
print ("go to S", 1)
x,y= drone.direction(1)
drone.update_location(x,y)
# print("x",drone.positionX,"y",drone.positionY)
#you should check state here too 
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
    drone.update_location(x,y)
    # print("x",drone.positionX,"y",drone.positionY)

    print("checking for update the state")
    drone.is_it_alone()

print("path", drone.path)
drone.update_state()
print( "drone state:", drone.state)



