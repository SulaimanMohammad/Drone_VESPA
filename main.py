
import sys
import os
import signal
from pathlib import Path
# Add the src directory to sys.path
root_dir = Path(__file__).resolve().parent
src_dir = root_dir / "src"
sys.path.append(str(src_dir))
from VESPA.VESPA_module import *
from VESPA.expansion import first_exapnsion,further_expansion
from VESPA.spanning import spanning
from VESPA.balancing import balancing

def interrupt(signal_num, frame):
    print("Interrupted!")
    global vehicle
    if vehicle is not None:
        vehicle.remove_attribute_listener('velocity', on_velocity)
        vehicle.mode = VehicleMode ("LAND")
        time.sleep(3) 
        vehicle.close()
        sys.exit()

logs= create_log_file() 
global vehicle
vehicle = connect (parse_connect(), wait_ready=False) # for simulation 
#vehicle = connect("/dev/serial0", baud= 921600,  wait_ready=False) # for raspberry pi
#vehicle = connect("/dev/ttyUSB0", baud= 57600,  wait_ready=False, rate=10) #for telemetry 
#vehicle.wait_ready(True, raise_exception=False) # for raspberry pi & telemetry only 
signal.signal(signal.SIGINT, interrupt)


hight=2
drone=Drone(0,0,2)
first_exapnsion(drone, vehicle)
spanning(drone)
balancing(drone, vehicle)

while drone.check_termination():
    further_expansion( drone, vehicle)
    spanning(drone,vehicle)
    # Since the irremovable drone as stuck in spanning, then it will retuen and try to performe balancing 
    if drone.state==Irremovable and drone.VESPA_termination.is_set():
        break
    # NO need to do same for border because VESPA_termination will be set in balancing and then while evaluation comes
    balancing(drone, vehicle )

drone.return_home(vehicle) 





