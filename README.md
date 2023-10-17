# Drone_VESPA

## Setup Raspberry pi
- Connect aspberry pi to pixhawk using Telemetry 2 port
- Configure raspberry pi to be able to communicate with pixhawk 

```bash
        git clone https://github.com/SulaimanMohammad/Drone_VESPA.git
        cd Drone_VESPA
``` 
```bash
        ./rpi_setup.sh
```
- Check for updates, create logs directory, set permissions 

```bash
        ./update_repo.sh
``` 

## Scripts Map 
- "drone_ardupilot.py": API to move take off, and move the drone 
- "unit_tests" contains implementation of drone_ardupilot 
- "expan.py", "spaning.py" are the pahses in VESPA Algoithm 
    To see more about this algorithm check the [simulation here](https://github.com/SulaimanMohammad/self-organized-uav)


## Run tests 
- All tests in unit_tests can be used to commuinicate with raspberry pi, telemetry and simulation 
- The test used in the algorithm of VESPA is  body_frame_move.py 
    - It uses the movement using Yaw , distance and PID 
    
    ```bash
        python3 body_frame_move.py
    ``` 

## Expansion phase
Here you can have acess [expansion UML](https://SulaimanMohammad.github.io/Drone_VESPA/.exp/expansion.png)


### Forming border communication 
**Propagation Indicator:** This represents the list that previous node used to construct the target. Essentially, it's the list of all neighbors of the previous node.

Given that at least two neighbors are shared between two consecutive centers of a hexagon, this helps to prevent sending messages to nodes that have already received the message from behind. It also prevents forming a closed loop due to backward communication.

**Target:** This is the result of the difference between the current neighbors and the neighbors of the previous node. Therefore, messages are only sent to neighbors that haven't received any messages before.


The node will check if the sender of the received message is in the Propagation Indicator. If not, this implies that the message is either coming for the first time or from the same direction taken by the previous neighbors. It will then save that Propagation Indicator, which represents from which direction the message arrived at this node. It will then forward the message to everyone. However, only the node with an ID equal to the target in the message will respond and may forward the message further.

Note: If a node receives a message from a different direction (as illustrated in the figure when node 9 receives the message), it will verify if the sender's ID is in its saved Propagation Indicator. If it's not, this indicates that the direction is different, and it should continue forwarding the message in the same direction. The remaining nodes will continue to receive and forward the message until it reaches node 1, closing the circle.

Note: The same process will occur for node 8, and it will forward the message in the opposite direction (not shown in the figure). In this manner, the first message that arrives at node 1 will initiate the broadcast, stopping all other nodes from forwarding messages.

This method guarantees the full circulation of the message. In case of any communication error, we still have a chance to complete the cycle due to messaging in both directions. Moreover, it's faster since the construction starts in both directions, making it more reliable.

Note: since any message will be received by all the nodes in the range, thus there is need to ignore messages 

- if the id of the node is not in the targets_id in the message then the message will be ignored
- if the id of the sender is in the Propagation Indicator then the message will be ignored that due to the fact that the message came from the same center of hexagon

![Alt text](https://raw.githubusercontent.com/SulaimanMohammad/Drone_VESPA/main/UML/Forming_border_communication.drawio.svg)

#### Breaking into a spot in front of border candidate

![Alt text](https://raw.githubusercontent.com/SulaimanMohammad/Drone_VESPA/main/UML/breaking_into_spot.drawio.svg)

Suppose drone 15 is located in the same spot as drone 13. In this scenario, drone 13 considers itself to be Owner since it has claimed the spot.

In such a case, drone 15 will continue moving. If the process of forming a border has already started, drone 15 will join the formation after it has already begun. If no action has been taken, two scenarios may arise:

1. If the messages have already reached drone 14, then upon completion of the broadcast circle, drone 14 will check its status. It will find that it is no longer a candidate for forming the border and will become free. However, this will result in a discontinuity in the formation of the border path, as the remaining drones will continue to serve as border nodes.
2. If the messages have not yet reached drone 14, there is another issue: there is no communication in place to detect any changes in the topology of the surroundings. In this stage, both drones 15 and 14 will remain candidates and will continue to pass messages. Drone 14 will not become free until a broadcast that includes checking the surrounding drones is received. This could result in unnecessary communication and an unexpected border path.

This scenario highlights potential issues that could arise, impacting both the communication efficiency and the reliability of the formed border.

### Solution

When a drone moves and claims a spot, determining that it is a border candidate, it will send a message to its neighbors to indicate that it is a border candidate and has occupied a spot in the neighborhood (e.g., drone 15 sends messages to drones 1, 11, and 14).

Upon receiving this message, the neighboring drones will check which spot is occupied and reassess their own status as border candidates (for example, drones 1 and 11 remain candidates, while drone 14 is no longer a candidate).

If a drone is no longer a candidate, two situations may arise:

- If no messages regarding border formation have arrived yet, the drone will become "Free" and wait for the expansion process to complete (e.g., drone 14 becomes Free).
- If the drone has already received one or more messages and saved the IDs of drones initiating messaging circles, it will send these saved IDs, along with its Propagation Indicator, to the newly arrived drone and become Free. The message will be tagged so that only the new arrival (in this case, drone 15) will consider it.

The impact of this procedure is twofold: if a broadcast message arrives from a specific border candidate, drone 15 will already have this information, enabling it to become a border drone. The Propagation Indicator is used to handle scenarios where messages are still circulating, thus facilitating more efficient communication and border formation.

## Spanning UML 
UML for the communication in the spanning phase of VESPA

![Alt text](https://raw.githubusercontent.com/SulaimanMohammad/Drone_VESPA/main/UML/spanning_process.drawio.svg)


## How to find tagret in the region of the drone
The drone will scan the region by taking a hexagon path with length depends on the camera setting.
For example based on the focal length and the high of the camera, the covrage will be calculate and define the path of the drone.

```bash
        scan_hexagon(vehicle-object, VESPA-object, orgin-hex-length ,camera_image_width, scan_time)
```

- It will start by calculating the distance between two successive hexagons that confine between them the size of the image that can be captured by the drone
- calcule Distance/2 beause the drone should be in the middle.
- scan the first region
    - go north (y) to V'1= (V1-distance/2)
    - move on the path that form hexagon
    - finish the loop
- scan the second region
    - from the last position (v'1) go down by distance to v''1=( V'1-distance)
    - move on the path and scan a hexagon
    - Note: the reason of moving by "distance" not "distance/2" because (distance/2) was used to go to the path of the drone, then the next path is on "distane" as you see in the fig.
- repeat until can't subtract (distance) which means no more regions need to be scanned.
- suppose the last hexagon was the one with (v'''1), then the drone can back to the original center
by going south (a-distance*i+distance/2)
![Alt text](https://github.com/SulaimanMohammad/Drone_VESPA/blob/main/.exp/coverage.png)
