import time 
import sys
import os
parent_directory = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# Add the parent directory to sys.path
sys.path.append(parent_directory)
from VESPA.headers_variables import *
import serial


def connect_xbee(xbee_serial_port, baud_rate, timeout=1):
    # Open a serial connection
    global ser
    ser = serial.Serial(port=xbee_serial_port, baudrate=baud_rate, timeout=timeout)

def send_msg(message):
    try: 
        """ Send a message via XBee. """
        ser.write(message)
        time.sleep(0.1)
    except:
        raise Exception("Thread send_msg Interrupt received, stopping...")   


message_buffer = bytearray()
def retrieve_msg_from_buffer(stop_flag):
    global message_buffer 
    while not stop_flag.is_set(): # Keep checking for a complete message or condition related to the phase is not set
        try: 
            # Read data from USB serial if available
            while ser.in_waiting > 0:
                byte = ser.read(1)
                message_buffer.extend(byte)
            
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
    return bytearray(b'')

def close_xbee_port():
    ser.close()

def clear_buffer():
    global message_buffer
    # Read and discard all available data
    while ser.in_waiting > 0:
        ser.read(ser.in_waiting)
    # Also clear the global message buffer
    message_buffer.clear()