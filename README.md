<p align="center">
  <img src="https://github.com/SulaimanMohammad/Drone_VESPA/blob/main/monitoring_interface/logos/VESPA_color.png" alt="VESPA Logo" width="300"/>
</p>

<h1 align="center">Vehicles (Drones) Spreading using Self-organized Parallel Algorithm</h1>

## Overview

This repository represents the VESPA Algorithm, designed for the collective operation of multiple drones. Each drone in the fleet is equipped with a Raspberry Pi and flight controllers. In this implementation, each drone is provided with a Pixhawk flight controller. The code for controlling the drone is written in Python using [DroneKit library](https://dronekit-python.readthedocs.io/en/latest/).

## Implementation of Non-GPS Navigation for Drones
The drones usally are contoled by Radio Transmitter Controllers or simple navigation using DroneKit which use GPS.
Many application including VESPA required different way of navigation based on a specific distance which is not supported in DroneKit.
The drone movement in VESPA no longer relies on GPS coordinates. Instead, it is programmed to navigate based on (Distance, Angle), ensuring precise movement crucial for autonomous vehicles. Unlike [ArduPilot](https://ardupilot.org/), which does not offer native support for such navigation, these instructions are implemented in the[drone_ardupilot.py](https://github.com/SulaimanMohammad/Drone_VESPA/blob/main/src/drone/drone_ardupilot.py) script. This script converts velocities from the NED frame to the body frame using Euler angles and employs a PID controller to maintain speed, calculate the traveled distance, and reduce velocity upon reaching the destination. This approach ensures smooth movement and prevents overshooting.


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
    For more details about changing the values [Change Characteristic](#change-characteristic)

### Raspberry pi Setup
To set up a fleet of N drones, follow these steps:

1. **Naming Convention**: Each drone's Raspberry Pi during burning the Raspberry Pi OS​ should be named `droneX`, where `X` ranges from 1 to N.
    - `drone1` is designated as the sink drone, which stays on top of the ground station and maintains connections with all other drones in the fleet.

2. **Raspberry Pi Setup**: Each drone's Raspberry Pi must be configured to connect with its respective flight controller.

    - Connect raspberry pi to pixhawk using Telemetry 2 port
    - Configure raspberry pi to be able to communicate with pixhawk
        ```bash
            sudo apt install git
            git clone https://github.com/SulaimanMohammad/Drone_VESPA.git
            cd Drone_VESPA/configuration
        ```
    - Configure RPi to be used with pixhawk and VESAP
        ```bash
            ./rpi_setup.sh
        ```
        This script call the followig scripts also:
        - **setup_drone_info**: Set variables needed for the drone, dimensions of drone are needed to be provided.
        - **create_VESPA_service**, **vespa**: Set VESPA to run as a service with shell commands
        - **update_repo**: Create logs directory, set permissions, and used to re-clone the repository


## Execute VESPA

### Hardware Requirements for VESPA Execution
To effectively execute the VESPA algorithm, the following hardware setup is required:

- Ground Control Station (GCS): One Xbee module needs to be connected to the local machine acting as the GCS.
- Fleet : A minimum of three vehicle units are needed, each connected to a Raspberry Pi to execute the VESPA algorithm.
    - One should be configured with `drone1` as sink
    - The rest two are required to form border in the algorithm

### Raspberry Pi
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

### Ground Controle Station (GCS)
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

## VESPA Operation Procedures

1. **Power On**: After powering on all the drones in the field, each drone will run the VESPA script as a service and wait for the GCS (Ground Control Station) to be launched on the local machine.
2. **GCS Initialization**: Once the GCS is launched, it will send a message to all the drones, prompting each drone to initialize the flight control and set all the necessary parameters.
3. **System Readiness Check**: The GCS will wait for signals from the drones indicating that the system is ready.
4. **Authorization for Movement**: When all systems are ready, the GCS will ask the user to authorize the start of VESPA movement.


## Running Scripts

There are three ways to execute the script:

1. **On Raspberry Pi**:
   - Ensure the Raspberry Pi is connected to the vehicle and configured.
   - Access the Raspberry Pi using SSH and run the script:
     ```bash
        python script.py
     ```

2. **On Local Machine**:
   - Before running the script, install the required libraries for MAVLink (do not use a virtual environment for Python):
     ```bash
        pip install mavproxy
        pip install dronekit
        pip install dronekit-sitl
     ```
    - **Using Telemetry Device**: The device is used to establish communication between the local machine and the vehicle directly (no need for a Raspberry Pi connection):
        ```bash
            python script.py telemetry
        ```
    - **Running in Simulation**: For more details on simulation see [Run simulation](#run-simulation)
        ```bash
            python script.py --connect ip:port
        ```


## Simulation

### Install simulation IDE
- Required libraries
   ```bash
        pip install mavproxy
        pip install dronekit
        pip install dronekit-sitl
        pip install pexpect
        pip install pysim
        python -m pip install empy
        pip install mavproxy
        pip install opencv-python  #for the map
    ```
- Follow the steps listed [here](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux) then [here ](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)

- Launch the Simulation, use the following command:
     ```bash
        python sim_vehicle.py -v ArduCopter -f quad --custom-location=Latitude,Longitude,Altitude,Heading  --console --map
    ```
    Latitude, Longitude, Altitude: Represents the position where the vehicle starts.

    Heading: The direction the vehicle is facing.

### Run simulation
#### With Raspberry Pi (preferred )

If you have multiple Raspberry Pis, each one can be considered a vehicle (it's a simulation, so no need for the RPi to be connected to Pixhawk) but as mentioned before minimum three Raspberry Pis.

In this setup, each RPi is configured and has its ID, which is needed in VESPA.

The following steps shoudl be done for each  Raspberry Pi:

1. **Connect RPi to Wi-Fi**: Ensure that RPi and local machine are connected to the same Wi-Fi network.

2. **Connect to each Raspberry Pi using SSH**.

3. **Stop Background Services**: Stop the service that needs a real vehicle to avoid conflicts, as only the simulation is needed.
     ```bash
        vespa stop
     ```

4. **Retrieve the IP Address of the Raspberry Pi**.

5. **Launch SITL on Your Local Machine**:
    - Open a new terminal and launch SITL (Software In The Loop) as follows:
    ```bash
        python sim_vehicle.py -v ArduCopter -f quad -I x --custom-location=Latitude,Longitude,Altitude,Heading --out=RPI_IP:Port --console --map
    ```
    - `RPI_IP`: IP address of Raspberry Pi, port: can be chosed as you want
    - `x`: Identifier of the simulation instance.

6. **Run Script on the Raspberry Pi**:
    - On the Raspberry Pi, run any script as follows:
     ```bash
        python script.py --connect RPI_IP:Port
     ```
    - Same port used to start STL should be used in running the script.

##### Notes:
- If the entire VESPA system is needed to be simulated whic needs XBee module that must be connected to each Raspberry Pi (for unit tests, Xbee is not required since those tests do not include commuinication )
- Each Raspberry Pi (simulated vehicle) needs its own SITL instance. For `N` Raspberry Pis, you need `N` instances of SITL (`-I [1:N]`).


#### Running Simulation Without Raspberry Pi (All on Local Machine)

Since the simulation is entirely done on the local machine, here are some important points to consider:

1. **Vehicle IDs**: VESPA requires an ID for each vehicle. Because there are no Raspberry Pis with unique IDs, you need to add an ID with each script execution.

2. **Xbee Modules**: Ensure Xbee modules are connected via USB.

3. **MAVLink Communication**: Communication with MAVLink in SITL will be done using localhost with different Port for each vehicle.
For each Vehicle you want to simulate should has it is proper port and STL instance:

    1. **Launch SITL Instance**: Launch SITL for the vehicle.
        ```bash
            python sim_vehicle.py -v ArduCopter -f quad -I x --custom-location=Latitude,Longitude,Altitude,Heading --out=127.0.0.1:Port_x --console --map
        ```
        - `127.0.0.1:Port_x`: Specifies the local host and port for communication.
        - `-I x`: Identifier for the simulation instance.

    2. **Run the Script**: Execute the script with the proper connection details:
        ```bash
            python script.py --connect 127.0.0.1:Port_x --id=x
        ```




## Run tests
- All tests in [unit_tests](https://github.com/SulaimanMohammad/Drone_VESPA/tree/main/src/drone/unit_tests) can be used  with raspberry pi, telemetry and simulation.

The tests are used to check that the drone is configured well and also for the reason of development and test modification, ( no need to have Xbee module to run these tests)
Each test can be run in one of the revious methods ( RPi , telemtry or simulation)
- [Calibration](https://github.com/SulaimanMohammad/Drone_VESPA/blob/main/src/drone/unit_tests/Calibration/measure_acc.py): contains 2 scripts
    - [measure_acc](https://github.com/SulaimanMohammad/Drone_VESPA/blob/main/src/drone/unit_tests/Calibration/measure_acc.py) used to move the drone and measures the acceleration and deacceleration
    - [extract](https://github.com/SulaimanMohammad/Drone_VESPA/blob/main/src/drone/unit_tests/Calibration/extract.py): use the data generated by measure_acc, filter them and return (acceleration and deacceleration) values that are necessary to move the drone using distance, angle instead of GPS coordination




## Compatibility and Versatility
The VESPA Algorithm is versatile and can be adapted to work with various vehicles, including different drone controllers. By modifying the implementation of movement functions (such as take_off, simple_goto_thread, move_using_coord, move_to_spot, and return_home), you can tailor the algorithm to suit different drone controllers. This flexibility allows for the integration of multiple drone types within the same fleet.

Additionally, VESPA is a general algorithm that can operate in heterogeneous systems. It can be used with completely different types of vehicles, such as rovers, cars, and planes.



## Change Characteristic
Most of the changes should be done in [Operational_data](https://github.com/SulaimanMohammad/Drone_VESPA/blob/main/src/Operational_Data.txt), thus the repository should be forked and changes made to the `Operational_data` file. Then, clone the updated data to the Raspberry Pi that is connected with Pixhawk (or change the file manually on each Raspberry Pi using SSH).
The [Operational_data](https://github.com/SulaimanMohammad/Drone_VESPA/blob/main/src/Operational_Data.txt) variables are:

1. Drone variables: related with actual drone:

    - **max_acceleration, max_deceleration**: Initial values can be changed using [Calibration](https://github.com/SulaimanMohammad/Drone_VESPA/blob/main/src/drone/unit_tests/Calibration/measure_acc.py) or kept the same. The drone will also adjust these dynamically while moving so no need to change anything after the first flight.
    - **defined_groundspeed**: Speed of the drone at the first movement commanded by the sink.
    - **lidar_scan**: Indicates whether to use lidar for object avoidance. Set to `True` if using lidar, otherwise `False`.
    - **speed_of_drone**: Speed of the drone in m/s after the first movement. It's recommended to keep this speed between 2 to 4 m/s, and 2 m/s if using lidar to ensure fast reactivity.
    - **drone_length, width, height, drone_id**: These will be defined when the Raspberry Pi configuration is launched.
    - **telemetry1_baudrate**: Baud rate for the telemetry device connection.
    - **telemetry2_baudrate**: Baud rate for Pixhawk where the Raspberry Pi is connected via UART. To change the value, use MAVLink and a framework like Mission Planner, update `SERIAL2_BAUD` in the parameter list, write the new value, and then update this file with the same value.

2. VESPA variables: related with VESPA algorithm:
    - **drone_id**: unique identifier of each vehicle, it is set up automatically by `setup_drone_info`, ( 0: GCS , 1: sink)
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

- **Xbee_module**: Module to send/read data using XBee (USB and GPIO)

- **Lidar**: Object Detection Using LiDAR and DBSCAN Algorithm. An ESP32 and Lidar need to be connected. [Repo here](https://github.com/SulaimanMohammad/Object_detection_Lidar).

- **Configuration**: Contains the bash scripts to configure the Raspberry Pi and also includes a script to get the hardware address of the Pixhawk.

- **Monitoring_interface**: Contains the HTML file to view the locations of targets.

- **Operational_data.txt**: Contains the acceleration and ID of the drone and all data needed through the algorithm.


## Experiment
