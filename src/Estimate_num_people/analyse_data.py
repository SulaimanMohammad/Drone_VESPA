allowable_gap = 50
def update_sequence_with_mac(sequences):
    A = {}  # Initialize the dictionary to store MAC prefixes as keys, with lists of sequence numbers as values
    for line in sequences.split('\n'):
        if line.strip():  # Check if the line is not empty
            mac, num_str = line.strip().split()  # Split MAC address and sequence number
            num = int(num_str)
            mac_prefix = mac.upper()[:8]  # Get the MAC address prefix

            to_replace = False  # Flag to check if the current num has replaced an element in A
            found= False
            # Use a temporary list to hold keys due to potential modification during iteration
            keys_list = list(A.keys())
            for key in keys_list:
                if mac_prefix==key:
                    found= True
                    for vale in range(0, len(A[key])):
                        if (num <= (A[key][vale] + allowable_gap)) and (num >= (A[key][vale] - allowable_gap)) :  # Compare with the last ele>
                            A[key][vale] = num  # Replace the last element with the new number
                            to_replace = True
                            break  # Stop searching once a replacement is made

                    # After checking all value and did not find close value appened
                    if not to_replace:  # If num didn't replace any element, add it as a new element with its MAC prefix
                        A[mac_prefix].append(num)  # Append the new number to the list
                        break # no need to check the rest of the list

            if not found: # mac_prefix not in A:
                A[mac_prefix] = []
                A[mac_prefix].append(num)
    return A

# Function to read data from a file and pass it to the update_sequence_with_mac function
def process_file(filename):
    with open(filename, 'r') as file:
        data = file.read()
        updated_sequence = update_sequence_with_mac(data)
        return updated_sequence

def detect_phones(filename, updated_sequence):
    with open(filename, 'r') as file:
        file_content = file.read()
    '''
    # Filter the list based on the presence of each MAC prefix in the file content
    filtered_mac_prefixes_non_phone = [prefix for prefix in updated_sequence if prefix in file_content]
    result_dict_non_phone = {prefix: len(num) for prefix, num in updated_sequence.items() if prefix in filtered_mac_prefixes_non_phone}
    sum_values_non_phone = sum(result_dict_non_phone.values())
    print("Number of non-phone devices:", sum_values_non_phone)
    '''
    # Filter the list based on absent of each MAC prefix in the file content, which means it is phone
    filtered_mac_prefixes = [prefix for prefix in updated_sequence if prefix not in file_content]
    result_dict = {prefix: len(num) for prefix, num in updated_sequence.items() if prefix in filtered_mac_prefixes}
    sum_values = sum(result_dict.values())
    return sum_values

def main():
    # File contains unique MACs
    script_dir = os.path.dirname(os.path.realpath(__file__))
    unique_mac = os.path.join(script_dir, 'unique.txt')
    # Read the data from the file and update the sequence
    updated_sequence = process_file(unique_mac)
    # File contains MACS of devices that are not phones
    Non_phones_mac = os.path.join(script_dir, 'Non_phones_macs.txt')
    numebr_of_phones= detect_phones( Non_phones_mac, updated_sequence)
    print( numebr_of_phones)

if __name__ == "__main__":
    main()
