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
## Spanning UML 
UML for the communication in the spanning phase of VESPA

![Alt text](https://github.com/SulaimanMohammad/Drone_VESPA/blob/main/.exp/spanning_process.png)


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
