import re
import os


speed =2

# Regular expression pattern to match the desired lines
pattern_time = re.compile(r"Time between two events: (\d+\.\d+) seconds")
pattern_array = re.compile(r"\[\s*(-?\d+\.\d+),\s*(-?\d+\.\d+),\s*(-?\d+\.\d+)\s*\]")

# Lists to store the extracted time and array values before and after "STOP"
time_values_acc = []
array_values_acc = []
time_values_decc = []
array_values_decc = []

# Flag to indicate whether "STOP" has been encountered
after_stop = False
# Flag to indicate whether a time value has been found in the previous line
time_found = False
# Variable to store the last found time value
last_time_value = None

# Open the file and extract the lines
with open('output.txt', 'r') as file:
    for line in file:
        # Check if the line contains the word "STOP"
        if "STOP" in line:
            after_stop = True  # Set the flag to True after encountering "STOP"
            continue

        # Check if the line matches the pattern for time
        match_time = pattern_time.search(line)
        if match_time:
            last_time_value = float(match_time.group(1))
            time_found = True  # Set the flag to True after finding a time value
            continue  # Skip to the next iteration

        # Check if the line matches the pattern for array
        match_array = pattern_array.search(line)
        if match_array and time_found:  # Check if a time value was found in the previous line
            array_value = [float(val) for val in match_array.groups()]
            # Append to the appropriate lists based on the after_stop flag
            (time_values_decc if after_stop else time_values_acc).append(last_time_value)
            (array_values_decc if after_stop else array_values_acc).append(array_value)
            time_found = False  # Reset the flag after processing an array value

# Output the extracted values
# print("Extracted time values acceleration:", time_values_acc)
# print("Extracted array values acceleration:", array_values_acc)
# print("Extracted time values deceleration:", time_values_decc)
# print("Extracted array values deceleration:", array_values_decc)


index_desired=0
#Find the index where the second element of the array is more than 2
for index, array in enumerate(array_values_acc):
    if array[1] >= speed: 
        index_desired= index
        break
else:
    # In case the speed is high and distance not enough to arrive to the set speed 
    index_desired= len(array_values_acc )-1

time_intervals=time_values_acc[:index_desired+1]

# Sum of all time intervals
total_time = sum(time_intervals)

# Initial and final velocities
V_initial = array_values_acc[0][1]  # units/s
V_final = array_values_acc[index_desired][1]  # units/s

# Overall Deceleration
overall_accceleration = (V_final - V_initial) / total_time
print ("total_time", total_time, "accceleration=" , overall_accceleration)


index_desired=0
#Find the index where the second element of the array is more than 2
for index, array in enumerate(array_values_decc):
    if array[1] <= 0.01: 
        index_desired= index
        break
else:
    # In case the speed is high and distance not enough to arrive to the set speed 
    index_desired= len(array_values_decc )-1

time_intervals=time_values_decc[:index_desired+1]
# Sum of all time intervals
total_time = sum(time_intervals)

# Initial and final velocities
V_initial = array_values_decc[0][1] # units/s
V_final = array_values_decc[index_desired][1]  # units/s

# Overall Deceleration
overall_Deceleration = (V_final - V_initial) / total_time
print ("total_time", total_time, "Deceleration=" , overall_Deceleration)

# Get the current working directory
current_dir = os.getcwd()
# Change to the parent directory
os.chdir("../..")
with open('Operational_Data.txt', 'w') as file:
        file.write(f'max_acceleration = {overall_accceleration}\n')
        file.write(f'max_deceleration = {overall_Deceleration}\n')
