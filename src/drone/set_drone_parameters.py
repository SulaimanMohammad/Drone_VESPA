from dronekit import connect, VehicleMode, APIException
import time
from drone.drone_ardupilot import *
def set_parameter(vehicle, parameter, value, retries=3, wait=2):
    """
    Set a parameter on the vehicle only if the current value is different from the desired value.
    """
    current_value = vehicle.parameters[parameter]
    if current_value != value:
        write_log_message(f"Current {parameter} is {current_value}, setting it to {value}.")
        for i in range(retries):
            try:
                vehicle.parameters[parameter] = value
                write_log_message(f"Attempt {i+1}: Set {parameter} to {value}")
                # Wait to ensure the parameter is set
                time.sleep(wait)
                # Verify if the parameter is set correctly
                if vehicle.parameters[parameter] == value:
                    write_log_message(f"{parameter} successfully set to {value}")
                    return True
                else:
                    write_log_message(f"{parameter} set failed, current value: {vehicle.parameters[parameter]}")
            except (APIException, KeyError) as e:
                write_log_message(f"Error setting parameter: {e}")
            time.sleep(wait)
        return False
    else:
        write_log_message(f"{parameter} is already set to the desired value: {value}")
        return True

def config_parameters(vehicle, drone):
    set_parameter(vehicle, 'RTL_ALT', int((drone.drone_alt)*100)) # RTL_ALT in cm 
    set_parameter(vehicle, 'RTL_SPEED', drone.defined_groundspeed*100) # RTL_SPEED cm/s