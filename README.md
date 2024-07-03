# Vehicle (Drone) Spreading using Self-organized Parallel Algorithm

This repository represents the VESPA Algorithm, designed for the collective operation of multiple drones. Each drone in the fleet is equipped with a Raspberry Pi and flight controllers. In this implementation, each drone is provided with a Pixhawk flight controller. The code for controlling the drone is written in Python using [DroneKit library](https://dronekit-python.readthedocs.io/en/latest/) .

## Setup

### Fleet Setup
To ensure proper operation of the VESPA Algorithm, each flight controller (such as Pixhawk) must be connected to a framework that uses MAVLink, such as Mission Planner, for configuration.

1. **Connect to Mission Planner**: 
   Connect each flight controller to Mission Planner or a similar MAVLink-compatible framework.

2. **Calibrate the Flight Controller**:
   - **Compass**: Calibrate the compass to ensure accurate heading information.
   - **IMU (Accelerometer)**: Calibrate the IMU to ensure accurate motion detection.
   - **ESC (Electronic Speed Controllers)**: Calibrate the ESCs for proper motor control.

3. **Upload Parameter List**:

    Upload the parameter list that defines the parameters needed for the algorithm. This includes settings such as baud rates and other values necessary for the algorithm to function correctly.

4. **Peripheral devices**
    - Connect Raspberry pi with Telemetry2 of Pixhawk using UART pins (Pin3 to RPI-Tx, Pin2 to RPI-Rx). [Telemetry2 pins map](https://docs.holybro.com/autopilot/pixhawk-6c/pixhawk-6c-ports#telem-2-port)
    - Connect Xbee module to Raspberry pi, methode of connection should be mentioned in Operational_data file
        - Using USB: set (Tx,Rx) in  Operational_data to None.
        - Using GPIO pins: The [Pigpio](https://github.com/joan2937/pigpio) library will be used to manage the serial connection.
            1. The used GPIOs Tx and Rx should be specified in the `Operational_data`, or you can use pins (Tx 23) and (RX 24) of Raspberry pi, which are already mentioned in the `Operational_data`.
            2. Connect (Xbee-Din to RPI-Tx , Xbee-Dout to RPI-Rx).

### Raspberry pi Setup
To set up a fleet of N drones, follow these steps:

1. **Naming Convention**: Each drone's Raspberry Pi should be named `droneX`, where `X` ranges from 1 to N. where drone1 is designated as the sink drone, which stays on top of the ground station and maintains connections with all other drones in the fleet.

2. **Raspberry Pi Setup**: Each drone's Raspberry Pi must be configured to connect with its respective flight controller.

- Connect raspberry pi to pixhawk using Telemetry 2 port
- Configure raspberry pi to be able to communicate with pixhawk
    ```bash
        sudo apt install git
        git clone https://github.com/SulaimanMohammad/Drone_VESPA.git
        cd Drone_VESPA/configuration
    ```
- Configure RPI to be used with pixhawk and VESAP
    ```bash
        ./rpi_setup.sh
    ```
    This script call the followig scripts also:
    - **setup_drone_info**: Set variables needed for the drone, dimensions of drone are needed to be provided
    - **create_VESPA_service**, **vespa**: Set VESPA to run as a service with shell commands
    - **update_repo**: Create logs directory, set permissions, and used to re-clone the repository


## Running the Code on Raspberry Pi
**Each drone will automatically start VESPA after booting, but the process will not begin until it receives a signal from the Ground Control Station.**

The `rpi_setup` script will create a VESPA service that runs in the background upon each boot of the algorithm.

Also shell commands are set up, with these options, you can easily manage the VESPA algorithm on your Raspberry Pi, ensuring efficient operation and maintenance.

```bash
        vespa [option]
        show:    Show real-time printing of the execution of the code.
        state:   See the state of the VESPA algorithm, running or not.
        stop:    Stop running the VESPA algorithm.
        clean:   Clean all old logs.
        restart: Restart running the VESPA algorithm.
        remove:  Remove the service from running after each boot.
        --help
```
Those command used in case a Raspberry Pi accessed by ssh protocol.

## Running Ground Controle Station (GCS)
The GCS is a local machine with an Xbee module connected to it using a USB port.

1. **Clone the Repository**:
   Clone the repository to a local machine

2. **Run the GCS Script**:
   Use the following command to run the GCS script:
   ```bash
        python GCS.py
   ```
3. Monitoring the data from all the drones that comes from the drones run the following.
    ```bash
        ./monitoring.sh
    ```

## Scripts Map
- ** **
```
├── Configuration
├── src/
│   ├── VESPA
│   │   ├── VESPA_module.py
│   │   ├── expansion.py
│   │   ├── spanning.py
│   │   ├── balancing.py
│   │   ├── headers_variables.py
│   │   ├── form_border_one_direction.py
│   ├── drone/
│   │   ├── drone_ardupilot.py
│   ├── unit_test
│   │   ├── Calibration
│   │   │   ├── measure_acc.py
│   │   │   └── extract.py
│   │   └── unit_tests...
│   ├── Xbee_module/
│   │   ├── xbee_pigpio.py
│   │   ├── xbee_usb.py
│   ├── Lidar/
│   ├── Operational_data.txt
└── monitoring_interface
```
- **VESPA**: It contains all the phases of VESPA Algorithm. To see more about this algorithm, check the [Simulation here](https://github.com/SulaimanMohammad/self-organized-uav).

- **Drone**: "drone_ardupilot.py" API takes off and moves at a specific angle and distance. The "unit_test" directory contains many tests to check the movement of the drone, including a calibration directory that moves the drone, measures the acceleration, and sets it in Operational_data as a starting point (optional).

- **Xbee_module**: Module to send/read data using XBee.

- **Lidar**: Object Detection Using LiDAR and DBSCAN Algorithm. An ESP32 and Lidar need to be connected. [Repo here](https://github.com/SulaimanMohammad/Object_detection_Lidar).

- **Configuration**: Contains the bash scripts to configure the Raspberry Pi and also includes a script to get the hardware address of the Pixhawk.

- **Monitoring_interface**: Contains the HTML file to view the locations of targets.

- **Operational_data.txt**: Contains the acceleration and ID of the drone and all data needed through the algorithm.


## Run tests
- All tests in unit_tests can be used to commuinicate with raspberry pi, telemetry and simulation
- The test used in the algorithm of VESPA is body_frame_move.py
    - It uses the movement using Yaw , distance and PID
    - To run it on physical drone after connecting with ssh to RP
    ```bash
        python body_frame_move.py
    ```
    - In case of using STL simulation
        - launch the simulation, check the ip by writing output in mavlink terminal
        - Edit connection methode in your tests [ vehicle = connect (parse_connect(), wait_ready=False) ]

     ```bash
        python body_frame_move.py --connect [your ip]
    ```

## Compatibility and Versatility
The VESPA Algorithm is versatile and can be adapted to work , any other drone controller by modifying the implementation of the movementsuch as (take_off,simple_goto_thread,move_using_coord,move_to_spot,return_home).
These functions can be modified to suit different drone controllers as needed.


## Expansion phase
![Alt text](https://raw.githubusercontent.com/SulaimanMohammad/Drone_VESPA/main/UML/expansion_process.drawio.svg)

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


## Balancing phase
UML for the communication in the Balancing phase of VESPA
![Alt text](https://raw.githubusercontent.com/SulaimanMohammad/Drone_VESPA/main/UML/balancing_process.drawio.svg)

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
