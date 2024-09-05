import time 
import sys
import os
import struct
from VESPA.headers_variables import *
import serial


def connect_xbee(xbee_serial_port, baud_rate, timeout=1):
    # Open a serial connection
    global ser
    ser = serial.Serial(port=xbee_serial_port, baudrate=baud_rate, timeout=timeout)

def position_sensitive_checksum(message):
    checksum = 0
    for index, byte in enumerate(message):
        # Multiply each byte by its position index (position + 1 to avoid multiplication by zero)
        checksum += (index + 1) * byte
    return checksum % 256

def appened_checksum(msg):
    # Calculate the checksum
    checksum = position_sensitive_checksum(msg[:-1])  # Exclude the newline character for checksum calculation
    checksum_byte = struct.pack('>B', checksum)

    # Insert the checksum before the newline character
    msg = msg[:-1] + checksum_byte + msg[-1:]
    return msg

def verify_checksum(msg):
    message_without_newline = msg[:-1]
    # Extract checksum from the message
    received_checksum = struct.unpack('>B', message_without_newline[-1:])[0]
    # Recalculate checksum for the message excluding the checksum byte itself
    calculated_checksum = position_sensitive_checksum(message_without_newline[:-1]) 
    if received_checksum == calculated_checksum: 
        return True
    else:
        return False 
    
def send_msg(msg):
    try: 
        """ Send a message via XBee. """
        # Add checksum to the message before sending it 
        msg= appened_checksum(msg)
        ser.write(msg)
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
                    break  # Exit the inner loop to read more data

                # Find the end of the message (newline character)
                newline_index = message_buffer.find(b'\n', header_index)
                if newline_index == -1:
                    # Newline not found, need more data
                    break  # Exit the inner loop to read more data

                # Extract the complete message
                complete_message = message_buffer[header_index:newline_index + 1]
                # Remove the processed message from the buffer
                message_buffer = message_buffer[newline_index + 1:]
                # Check if the message complies with the required format  
                if complete_message and complete_message[0] in headers_ascii_values and complete_message.endswith(b'\n'):
                    if verify_checksum( complete_message): 
                        original_message = complete_message[:-2] + complete_message[-1:]
                        return original_message
                    else: 
                        break 
            
            # Short sleep to prevent high CPU usage
            time.sleep(0.05)

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