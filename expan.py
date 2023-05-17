from enum import Enum
from math import sqrt
import random

sq3=sqrt(3)
a= 20 # drone range 
C=100
eps=20


class DroneState(Enum):
    s1 = 0
    s2 = 1
    IRREMOVABLE = 2
    BORDER_AND_IRREMOVABLE = 3  

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
    [-(sq3 / 2) * a, (3.0 / 2.0) * a],  # s3
    [-sq3 * a, 0],             # s4
    [-(sq3 / 2.0) * a, -(3.0/ 2.0) * a], # s5
    [(sq3 / 2.0) * a, -(3.0 / 2.0) * a]   # s6
]

def set_a( val):
    global a
    a= val
    update_DIR_VECTORS()



formula_dict = {
    "s0": "sqrt(DxDy2)",
    "s1": "sqrt(DxDy3a2 + a * aDx)",
    "s2": "sqrt(DxDy3a2 + a * (sqDx + (3 * Dy)))",
    "s3": "sqrt(DxDy3a2 + a * (3 * Dy - aDx))",
    "s4": "sqrt(DxDy3a2 - aDx + 3 * a * a)",
    "s5": "sqrt(DxDy3a2 - a * (sqDx + (3 * Dy)))",
    "s6": "sqrt(DxDy3a2 - a * (3 * Dy - sqDx))"
}

class Drone: 
    def __init__(self, x,y):
        self.positionX=x
        self.positionY=y
        self. min_distance_dicts=[] # nigboor close to the sink 
        self.s_list = []
        self.Priority_to_go=[]
        self.num_neigboors = 6
        for i in range(0, self.num_neigboors+1):
            s = {"name": "s" + str(i), "distance": 0, "priority": 0, "occupied": False, "drones_in": 0}
            self.s_list.append(s)
        
        # s1={"name":"s1", "distance":0, "priority":0, "occupied": False, "drones-in":0}
        # s2={"name":"s2","distance":0, "priority":0, "occupied": False, "drones-in":0}
        # s3={"name":"s3","distance":0, "priority":0, "occupied": False, "drones-in":0}
        # s4={"name":"s4","distance":0, "priority":0, "occupied": False, "drones-in":0}
        # s5={"name":"s5","distance":0, "priority":0, "occupied": False, "drones-in":0}
        # s6={"name":"s6","distance":0, "priority":0, "occupied": False, "drones-in":0}

    def calculate_neigboors_dis(self):
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
        for s in self.s_list:
            # for now i will have it from the Stdin 
            num_dron = int(input("Enter number of drone at "+s["name"]+" :"))
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
        return DIR_VECTORS[dir][0], DIR_VECTORS[dir][1]



