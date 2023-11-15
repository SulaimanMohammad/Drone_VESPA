'''
These will be used in the building and decoding message
'''
#Exchanging data Headers
Movement_command= 'M'
Calibration= 'C'
Demand_header= 'D'
Response_header= 'R'
ACK_header= "K"
# Expansion Headers
Expan_header= 'E'
Arrival_header= 'A'
Inherit_header= 'I'
Forming_border_header= 'F'
#Spanning Headers
Spanning_header= 'S'
Target_coordinates_header= 'T'
# Blancing Headers
Local_balance_header='L'
Guidance_header= 'G'
Balance_header= 'B'
Algorithm_termination_header="O"

# Create an array of headers
headers = [
    Movement_command, Calibration, Demand_header, Response_header,ACK_header,
    Expan_header, Arrival_header, Inherit_header, Forming_border_header,
    Spanning_header, Target_coordinates_header, Local_balance_header,
    Guidance_header, Balance_header, Algorithm_termination_header
]

# Generate an array of ASCII values to be used in retiveing messages 
headers_ascii_values = [ord(header) for header in headers]
