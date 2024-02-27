from pathlib import Path
import re
import importlib

'''
These will be used in the building and decoding message
'''
#Exchanging data Headers
Movement_command= 'M'
Calibration= 'C'
Demand_header= 'D'
Response_header= 'R'
ACK_header= 'K'
# Expansion Headers
Expan_header= 'E'
Arrival_header= 'A'
Inherit_header= 'I'
Forming_border_header= 'F'
Verify_border_header='V'
#Spanning Headers
Spanning_header= 'S'
Target_coordinates_header= 'T'
# Blancing Headers
Local_balance_header='L'
Guidance_header= 'G'
Balance_header= 'B'
Algorithm_termination_header='O'

# Create an array of headers
headers = [
    Movement_command, Calibration, Demand_header, Response_header,ACK_header,
    Expan_header, Arrival_header, Inherit_header, Forming_border_header,Verify_border_header,
    Spanning_header, Target_coordinates_header, Local_balance_header,
    Guidance_header, Balance_header, Algorithm_termination_header
]

# Generate an array of ASCII values to be used in retiveing messages 
headers_ascii_values = [ord(header) for header in headers]

def read_vars_from_file():
        # Current script directory
        current_dir = Path(__file__).resolve().parent
        # Path to the parent directory
        parent_dir = current_dir.parent
        global Operational_Data_path
        Operational_Data_path = parent_dir/'Operational_Data.txt'

        with open(Operational_Data_path, 'r') as file:
            for line in file:
                match = re.match(r"(\w+)\s*=\s*([^#]+)", line)
                if match:
                    var_name, value = match.groups()
                    if var_name in ["max_acceleration", "max_deceleration"]:
                        continue
                    else:
                        try:
                            global_vars[var_name] = eval(value.strip(), {}, global_vars)
                        except NameError:
                            global_vars[var_name] = value.strip()
global_vars = {}
def load_vars():
    read_vars_from_file() # Include set id and xbee range "a" 
    globals().update(global_vars)

def get_global_vars():
    return global_vars

def set_env(globals_dict):
    load_vars()
    global_vars = get_global_vars()
    # Determine which XBee module to use and set 'uart'
    if Tx == None and Rx == None:
        xbee_module = importlib.import_module('Xbee_module.xbee_usb')
        global_vars['uart'] = True  # Set 'uart' to True for USB
    else:
        xbee_module = importlib.import_module('Xbee_module.xbee_pigpio')
        global_vars['uart'] = False  # Set 'uart' to False for GPIO

    globals_dict.update(global_vars)
    globals_dict.update(xbee_module.__dict__)
