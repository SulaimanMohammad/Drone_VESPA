from enum import Enum
from math import sqrt
import random
import copy

sq3=sqrt(3)
a= 20 # drone range 
C=100
eps=20


class DroneState(Enum):
    Free = 0
    BORDER_AND_IRREMOVABLE = 1
    IRREMOVABLE = 2


class Direction(Enum):
    s0 = 0
    s1 = 1
    s2 = 2
    s3 = 3
    s4 = 4
    s5 = 5
    s6 = 6

DIR_VECTORS = [
    [0, 0],                             # s0 // don't move, stay
    [(sq3 * a), 0],            # s1
    [(sq3 / 2.0) * a, (3.0 / 2.0) * a],   # s2
    [-(sq3 / 2) * a, (3.0 / 2.0) * a],  # s3
    [-sq3 * a, 0],             # s4
    [-(sq3 / 2.0) * a, -(3.0/ 2.0) * a], # s5
    [(sq3 / 2.0) * a, -(3.0 / 2.0) * a]   # s6
]

def update_DIR_VECTORS():
    global DIR_VECTORS
    DIR_VECTORS = [
    [0, 0],                             # s0 // don't move, stay
    [(sq3 * a), 0],            # s1
    [(sq3 / 2.0) * a, (3.0 / 2.0) * a],   # s2
    [-(sq3 / 2.0) * a, (3.0 / 2.0) * a],  # s3
    [-sq3 * a, 0],             # s4
    [-(sq3 / 2.0) * a, -(3.0/ 2.0) * a], # s5
    [(sq3 / 2.0) * a, -(3.0 / 2.0) * a]   # s6
]

def set_a( val):
    global a
    a= val
    update_DIR_VECTORS() # changing of a should change the direction distances

formula_dict = {
    "s0": "sqrt(DxDy2)",
    "s1": "sqrt(DxDy3a2 + a * aDx)",
    "s2": "sqrt(DxDy3a2 + a * (sqDx + (3 * Dy)))",
    "s3": "sqrt(DxDy3a2 + a * (3 * Dy - sqDx))",
    "s4": "sqrt(DxDy3a2 - (aDx*a))",
    "s5": "sqrt(DxDy3a2 - a * (sqDx + (3 * Dy)))",
    "s6": "sqrt(DxDy3a2 - a * (3 * Dy - sqDx))"
}

State= [ 
    "FREE",
    "Alone",
    "BORDER", 
    "IRRMOVABLE",
    "BORDER & IRRMOVABLE"
]

class Drone: 
    def __init__(self, x,y):
        self.positionX=x
        self.positionY=y
        self.state="NONE"
        self.path=[]
        self.a=a
        self. min_distance_dicts=[] # nigboor close to the sink 
        self.s_list = []
        self.Priority_to_go=[]
        # init s0 and it will be part of the spots list 
        self.s_list=[{"name": "s" + str(0), "distance": 0, "priority": 0, "occupied": False, "drones_in": 1}]
        # save the first spot which is s0 the current place of the drone 
        # spot is another name of s_list[0] so any changes will be seen in spot
        self.spot= self.s_list[0]
        self.num_neigboors = 6
        for i in range(1, self.num_neigboors+1):
            s = {"name": "s" + str(i), "distance": 0, "priority": 0, "occupied": False, "drones_in": 0}
            self.s_list.append(s)
        

    def calculate_neigboors_dis(self):
        
        # DxDy2 = round((self.positionX * self.positionX) + (self.positionY * self.positionY),2)
        # DxDy3a2 = round(DxDy2 + 3 * a * a,2)
        # sqDx = round(sq3 * self.positionX,2)
        # aDx = round((2*sq3) * self.positionX,2)
        # Dy= round(self.positionY,2)
        DxDy2 = (self.positionX * self.positionX) + (self.positionY * self.positionY)
        DxDy3a2 = DxDy2 + 3 * a * a
        sqDx = sq3 * self.positionX
        aDx = (2 * sq3) * self.positionX
        Dy= self.positionY
        #TODO you sshould consider a situation what inside the formaula is negative 
        for s in self.s_list:
            formula = formula_dict.get(s["name"])
            if formula:
                distance = eval(formula, {'sqrt': sqrt, 'DxDy2': DxDy2, 'DxDy3a2': DxDy3a2, 'a': a, 'aDx': aDx, 'sqDx': sqDx, 'Dy': Dy})
                s["distance"] = distance


    def check_drones_in_neigboors(self):
        #TODO it is about sending signal and recive it to count the number in each spot 
        # The idea the drone will recive signal from the nigboors the signal is coordinates 
        #of each drone and compare it, and since all the movement on 6 nigbor and the reive about a 
        # thn the messages usally are only from niegboor 
        # so it will compare with the posssible coordiantes of 
        # nigboor with respect to the sink( because can not compare with respect to the drone that recive the signal)
        # TODO in each of the spot s1-s6 creat x,y the coordinates with the respect to the sink then 
        # using x2+y2 find the distance 
            # no need for that because each drone will have it is own position calculated far from the sink 
        
        for s in self.s_list:
            # for now i will have it from the Stdin 
            num_dron = int(input("\t Enter number of drone at "+s["name"]+" :"))
            s["drones_in"] = int(num_dron)
            if num_dron > 0:
                s["occupied"]= True


    def update_location(self,x,y ):
        self.positionX =self.positionX + x#DIR_VECTORS[dir][0]# add the value not assign because it is movement 
        self.positionY = self.positionY+ y #DIR_VECTORS[dir][1]
        return [self.positionX, self.positionY]
    

    def findMinDistances_niegboor(self):
        min_distance = min(self.s_list, key=lambda x: x["distance"])["distance"]
        self.min_distance_dicts =[s["name"] for s in self.s_list if s["distance"] == min_distance]
        #self.min_distance_dicts = [s for s in self.s_list if s["distance"] == min_distance]

    def setPriorities(self):
        denom= 4.0 * self.s_list[0]["distance"] # 4*distance of s0 from sink 
        self.findMinDistances_niegboor()
        for s in self.s_list:
            if not s["occupied"]: # free spot 
                s["priority"]= s["distance"]* C /denom
                #print("s["name"],s["priority"] )

            else: # s is occupied 
                if s["name"] in self.min_distance_dicts: #close to sink 
                    s["priority"]= float("inf") 
                else: # random float between [w*c+eps, w+1*c[
                    s["priority"]= random.uniform(s["drones_in"]* C + eps, (s["drones_in"]+1)*C)

    
    def findPriority(self):
        min_Priority = min(self.s_list, key=lambda x: x["priority"])["priority"]
        self.Priority_to_go =[s["name"] for s in self.s_list if s["priority"] == min_Priority]
        
        #print(int(self.Priority_to_go[0][1:])) #exteract only the number of nigboor  
        return int(self.Priority_to_go[0][1:])


    def direction(self, dir):
     # the north is the y access in the calaulation of the hex and the east is the x 
     # example reived x=a, y=b 
     # then the move to the nourth by the value of b ( y in calculation access)
        self.path.append(dir) # add the direcition was chose 
        return DIR_VECTORS[dir][0], DIR_VECTORS[dir][1]

    
    def is_it_alone(self):
        self.check_drones_in_neigboors()
         #if s0 where the drone is conatins only the drone ( droen alone) 
         # the drone should be alone to be free 
        print("drone in s0", self.spot["drones_in"] )
        if self.spot["drones_in"]==1: # the drone is alone
            self.state="Alone" 
        print("drone state in s0", self.state )
    
    def update_state(self):
        if self.state=="Alone" : # the drone is alone so see if it is free or border ot irrmovable
            counter=0
            for s in self.s_list:
                if s["drones_in"] >=1: 
                    counter= counter +1 
                    continue
                else: 
                    break
            print("counter", counter)
            if counter==7: # all neighboor are occupied including the drone itself 
                self.state="FREE"
            else: #drone is alone but not sourrounded by drones   
            #TODO border just if it doent have niegboor on th path of the expansion 
            # NOTE :  after yann he said it is not possible to have more than one drone
            # here i need to verfiy whhat is should be 
            #because if it is border then that means it will do deal_state 
                dominated_direction= self.expansion_direction() #find the path that the drone take most of the time for the expansion 
                if self.is_it_border(dominated_direction):
                    self.state="BORDER"  
            

    def expansion_direction(self):
            occurance=[0,0,0,0,0,0] # s0-s6
            for i in range(0,len(self.path)):
                occurance[self.path[i]] +=1
                if i==len(self.path):
                    occurance[self.path[i]] +=2 # more weight to the last step
            return occurance.index(max(occurance)) #return the direction that has been taken most of time 
    
    def is_it_border(self, dom_dir):
        border=False
        unoccupied_neigboors=[]
        #self.check_drones_in_neigboors() # no need=to check agian it is already done in update state 
        for s in self.s_list:
            if s["occupied"]==False: #if spot is not occupied
                spot=  int(s["name"][1:]) #extract only the number 
                unoccupied_neigboors.append(spot) 

        if dom_dir==1:
            specific_spots=[2,1,6]            
        elif dom_dir==2:
            specific_spots=[1,2,3]
        elif dom_dir==3:
            specific_spots=[2,3,4]
        elif dom_dir==4:
            specific_spots=[3,4,5]
        elif dom_dir==5:
            specific_spots=[4,5,6]
        elif dom_dir==6:
            specific_spots=[5,6,1]
        
        print("dom_dir", dom_dir)
        if all(num in unoccupied_neigboors for num in specific_spots): # see if all the specific_spots are not occupied  
            border= True
        

    # terminate the stage 
    # if the drone is border it should start communicating 
    def deal_satae(self):
        if self.spot["state"]== "border": # nothing to do if you are free
            print (" i am border time to check  ") 
            # your massage sould be id@state

        else: # deone is not border so it should do nothing 
            pass


    # any messges recived should be buffered 
    def react_on_recived_msg(self):
        msg="id@state" # it should be  recived from a drone
        #if the recived message is a msg send by this drone befor 
        id, state = msg.split('@')
        if id == self.id and state== "border": 
            # expansion is done 
            print(" done expan ")
            '''
            The drone then sends a broadcast message
            indicating the end of the expansion phase and wait for ξ(D)
            time correlated to the diameter of the expansion area.
            '''
        else:
            if self.spot["state"]=="free": # drone is free should drop the message
                pass # do nothing 
            elif self.spot["state"]=="border":
                #redirect it 
                pass
