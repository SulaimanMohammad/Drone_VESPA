import dronekit_sitl
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time


# Get the parent directory path
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
from drone_ardupilot import *

vehicle=drone_connect()

@vehicle.on_attribute('mode')
def decorated_mode_callback(self, attr_name, value):
    # `attr_name` is the observed attribute (used if callback is used for multiple attributes)
    # `attr_name` - the observed attribute (used if callback is used for multiple attributes)
    # `value` is the updated attribute value.
    print(" CALLBACK: Mode changed to", value)

# When the function returns the app can continue in GUIDED mode or switch to AUTO mode to start a mission.
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    if vehicle.mode.name == "INITIALISING":
        print ("initialise the vehicle") 

    print("Basic pre-arm checks") 
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print ("Waiting for vehicle to initialise...")
        time.sleep(2)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print ("Waiting for arming...")
        time.sleep(1)

    print ("Taking off starts") 
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    #takeoff is asynchronous and can be interrupted if another command arrives before it reaches the target altitude so
    # check that arrived by reading the alitude relative from the ground 
    while True:
        print (" Altitude: ", vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:  # arrived to 95% of the altitude s
            print ("Reached target altitude")
            break
        time.sleep(1)

  
def location_callback(self, attr_name, value):
        print ("Location (NED): ", value)


#states 
print ("Autopilot Firmware version: %s" % vehicle.version)
#print ("Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp)
print ("Global Location: %s" % vehicle.location.global_frame)
print ("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)

# MAIN PROGRAM ----
print ("Local Location after arming : %s" % vehicle.location.local_frame)    #NED
# --- Here you can do all your magic....
# vehicle is an instance of the Vehicle class
time.sleep(40)
print ("after moving Local Location: %s" % vehicle.location.local_frame)    #NED
print ("Attitude: %s" % vehicle.attitude)
print ("Velocity: %s" % vehicle.velocity)
print ("GPS: %s" % vehicle.gps_0)
print ("Groundspeed: %s" % vehicle.groundspeed)
print ("Airspeed: %s" % vehicle.airspeed)
print ("Gimbal status: %s" % vehicle.gimbal)
print ("Battery: %s" % vehicle.battery)
print ("EKF OK?: %s" % vehicle.ekf_ok)
print ("Last Heartbeat: %s" % vehicle.last_heartbeat)
print ("Rangefinder: %s" % vehicle.rangefinder)
print ("Rangefinder distance: %s" % vehicle.rangefinder.distance)
print ("Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
print ("Heading: %s" % vehicle.heading)
print ("Is Armable?: %s" % vehicle.is_armable)
print ("System status: %s" % vehicle.system_status.state)
print ("Mode: %s" % vehicle.mode.name)    # settable
print ("Armed: %s" % vehicle.armed)    # settable
#--- Coming back.

print ("befor moving Local Location: %s" % vehicle.location.local_frame)    #NED
vehicle.add_attribute_listener('location.local_frame', location_callback)

vehicle.close()
