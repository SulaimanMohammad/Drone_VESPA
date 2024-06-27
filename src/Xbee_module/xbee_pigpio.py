import pigpio
import time 
from VESPA.headers_variables import *
import threading
send_lock = threading.Lock()

def connect_xbee(TX,RX,baud_rate_set):
    global tx_pin
    tx_pin=TX
    global rx_pin
    rx_pin=RX
    global baud_rate
    baud_rate= baud_rate_set
    
    # Open a serial connection
    global pi
    pi = pigpio.pi()
    if not pi.connected:
        raise Exception("Could not connect to pigpio daemon")
    # Set up the TX and RX pins for bit bang reading
    pi.set_mode(rx_pin, pigpio.INPUT)
    try:
        pi.bb_serial_read_open(rx_pin, baud_rate, 8)  # Try to open RX pin with a baud rate
    except pigpio.error as e:
        print(f"Error opening RX pin initially: {e}")
        try: 
            print("clear the buffer first")
            clear_buffer() # clear the buffer befor 
            pi.bb_serial_read_close(rx_pin)  # Close any existing serial read on the pin
            pi.bb_serial_read_open(rx_pin, baud_rate, 8)  # Open RX pin with a baud rate, 8: the number of data bits per character
        except pigpio.error as e:
            print(f"Error reopening RX pin: {e}")
            close_xbee_port()
            sys.exit(1)  # Exit the program with a non-zero status

    pi.set_mode(tx_pin, pigpio.OUTPUT)
    pi.wave_clear() # Clear any existing waveforms before sending the first message
    time.sleep(0.5) # wait until all set 
    send_msg('test'.encode() ) # Warm up message 

def send_msg(msg):
    with send_lock:
        try: 
            # Check if the message is empty or not in a byte-like format
            if not msg or not isinstance(msg, (bytes, bytearray)):
                print("Error: Message is empty or not in byte format.")
                return

            pi.wave_clear()  # Clear any existing waveforms
            if pi.wave_get_micros() > 0:  # Check if there's any data in the buffer
                pi.wave_clear()  # Clear it again to be sure

            pi.wave_add_serial(tx_pin, baud_rate, msg)  # Add a new waveform
            # wave_id = pi.wave_create()  # Create the waveform
            max_attempts = 10  # Maximum number of attempts to create the waveform
            wave_id = None
            for attempt in range(max_attempts):
                try:
                    wave_id = pi.wave_create()  # Attempt to create the waveform
                    if wave_id >= 0:
                        break  # Waveform created successfully, exit the loop
                except pigpio.error as e:
                    print(f"Attempt {attempt + 1} failed with error: {e}", " message was",msg )
                    time.sleep(0.05)  # Wait for 0.05 seconds before the next attempt

            # Check if the waveform was created successfully after all attempts
            if wave_id is None or wave_id < 0:
                print("Error: Failed to create waveform after multiple attempts.")
                return

            pi.wave_send_once(wave_id)  # Send the waveform
            while pi.wave_tx_busy():  # Wait until the waveform is sent
                time.sleep(0.05)
        except:
            raise Exception("Thread send_msg Interrupt received, stopping...")

message_buffer = bytearray()
def retrieve_msg_from_buffer(stop_flag):
    '''
    This function will continue reading and will break only when data is available in the way the listener will process the message upon arrival 
    For that, the outer loop will continue until the listener stops which is controlled by the flag and 
    break when data is available and the next iteration of 
    the listener will recall this loop again to trigger the listener only when data is available 
    '''
    global message_buffer
    while not stop_flag.is_set(): # Keep checking for a complete message or condition related to the phase is not set
        try: 
            # Read available data
            (count, data) = pi.bb_serial_read(rx_pin)
            if count:
                message_buffer.extend(data)

            # Process buffer if it has data
            while len(message_buffer) > 0:
                # Search for a valid header
                header_index = -1
                for i in range(len(message_buffer)):
                    if message_buffer[i] in headers_ascii_values:
                        header_index = i
                        break

                # If a valid header is not found, clear the buffer and return
                if header_index == -1:
                    message_buffer.clear()
                    return bytearray(b'')

                # Find the end of the message (newline character)
                newline_index = message_buffer.find(b'\n', header_index)
                if newline_index == -1:
                    # Newline not found, need more data
                    break  # Exit the inner loop to read more data

                # Extract the complete message
                complete_message = message_buffer[header_index:newline_index + 1]
                # Remove the processed message from the buffer
                message_buffer = message_buffer[newline_index + 1:]

                # Return the complete message
                return complete_message

            # Short sleep to prevent high CPU usage
            time.sleep(0.1)
        
        except:
            raise Exception("Thread retrieve_msg_from_buffer Interrupt received, stopping...")

    return bytearray(b'') # Return empty object so it can be recognized as not part of the headers array 

buffer_lock = threading.Lock()
def clear_buffer():
    global message_buffer
    with buffer_lock:
        message_buffer.clear()
        
        # Clear the pigpio serial read buffer
        try:
            while True:
                (count, data) = pi.bb_serial_read(rx_pin)
                if count == 0:
                    break  # Exit the loop if there's no more data to read
        except pigpio.error as e:
            print(f"Error while clearing pigpio buffer: {e}")

def close_xbee_port():
    global pi
    global rx_pin
    (count, data) = pi.bb_serial_read(rx_pin)
    if count > 0 or data != b'': # Check if bit bang serial read is open
        pi.bb_serial_read_close(rx_pin)
        pi.wave_clear()  # Clear any waveforms
    pi.stop()
    time.sleep(3) # Time to close the port 
