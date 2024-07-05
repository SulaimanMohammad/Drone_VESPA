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
    - Connect Raspberry pi with Telemetry2 of Pixhawk using UART pins `(Pin3 to RPI-Tx, Pin2 to RPI-Rx)`. [Telemetry2 pins map](https://docs.holybro.com/autopilot/pixhawk-6c/pixhawk-6c-ports#telem-2-port)
    - Connect Xbee module to Raspberry pi, methode of connection should be mentioned in Operational_data file
        - Using USB: set (Tx,Rx) in  Operational_data to None.
        - Using GPIO pins: The [Pigpio](https://github.com/joan2937/pigpio) library will be used to manage the serial connection.
            1. The used GPIOs Tx and Rx should be specified in the [Operational_data](https://github.com/SulaimanMohammad/Drone_VESPA/blob/main/src/Operational_Data.txt), or you can use pins (Tx 23) and (RX 24) of Raspberry pi, which are already mentioned in the `Operational_data`.
            2. Connect `(Xbee-Din to RPI-Tx , Xbee-Dout to RPI-Rx)`.
    For more details about changing the values [Change Characteristic](#change_characteristic)

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

## Running process

1. After powering on all the drones in the field, each drone will run the VESPA script and wait for the GCS (Ground Control Station) to be launched on the local machine.
2. Once the GCS is launched, it will send a message to all the drones, prompting each drone to initialize the flight control and set all the necessary parameters.
3. The GCS will wait for a signal from the drones indicating that the system is ready.
4. When all systems are ready, the GCS will ask the user to authorize the start of VESPA movement.


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



## Change Characteristic
Most of the changes should be done in [Operational_data](https://github.com/SulaimanMohammad/Drone_VESPA/blob/main/src/Operational_Data.txt), thus the repository should be forked and changes made to the `Operational_data` file. Then, clone the updated data to the Raspberry Pi that is connected with Pixhawk (or change the file manually on each Raspberry Pi using SSH).
The [Operational_data](https://github.com/SulaimanMohammad/Drone_VESPA/blob/main/src/Operational_Data.txt) variables are:

1. Drone variables: related with actual drone:

    - **max_acceleration, max_deceleration**: Initial values can be changed using [Calibration](https://github.com/SulaimanMohammad/Drone_VESPA/blob/main/src/drone/unit_tests/Calibration/measure_acc.py)or kept the same. The drone will also adjust these dynamically while moving so no need to change anything after the first flight.
    - **defined_groundspeed**: Speed of the drone at the first movement commanded by the sink.
    - **lidar_scan**: Indicates whether to use lidar for object avoidance. Set to `True` if using lidar, otherwise `False`.
    - **speed_of_drone**: Speed of the drone in m/s after the first movement. It's recommended to keep this speed between 2 to 4 m/s, and 2 m/s if using lidar to ensure fast reactivity.
    - **drone_length, width, height, drone_id**: These will be defined when the Raspberry Pi configuration is launched.
    - **telemetry1_baudrate**: Baud rate for the telemetry device connection.
    - **telemetry2_baudrate**: Baud rate for Pixhawk where the Raspberry Pi is connected via UART. To change the value, use MAVLink and a framework like Mission Planner, update `SERIAL2_BAUD` in the parameter list, write the new value, and then update this file with the same value.

2. VESPA variables: related with VESPA algorithm:
    - **eps, C**: Preferably kept unchanged.
    - **multiplier**: Set to 100. A higher value increases the precision of floating-point numbers in messages.
    - **exchange_data_latency**: Approximation of the time until the message is sent and processed in each XBee.
    - **spacing**: Space between each drone to maintain a safe zone. Higher precision barometers and GPS reduce this value (preferably start higher than 0.5 m).
    - **use_GPS**: Set to `True` if GPS will be used, `False` if flying without GPS. Flying without GPS requires additional equipment on the drone, such as computer vision.
    - **xbee_range**: The range of the XBee used.
    - **Xbee_change_range**: Set to `False` to use the provided XBee range in datasheet. Set to `False` if calibration of the XBee value is needed at the beginning of the movement.
    - **xbee_proficiency_ratio**: Actual XBee range used in VESPA, which is less than 1 to ensure operation within a safe range and avoid losing connection.
    - **ref_alt_ratio**: The height of the drone, considering that the drone will ascend slightly upon takeoff.

3. Xbee variables: related to the Xbee module that is connected with Raspberry pi.
    - **xbee_serial_port**: The serial port for the XBee if connected via USB. Usually, this doesn't need to be changed unless additional peripherals are connected to the Raspberry Pi.
    - **Tx, Rx**: GPIO numbers for the XBee connection if not connected via USB. Set these variables to `None` if the XBee is connected via USB.
    - **baud_rate**: Baud rate of the communication between the XBee and Raspberry Pi (9600 is preferred with `pigpio`).


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


