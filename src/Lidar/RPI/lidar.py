import serial
import threading
import queue
import time
import re  

def start_observer(hostVehicle, output_queue, read_lidar,emergecy_stop,data_ready, ref_alt):
    observer_thread = threading.Thread(target=observe_env, args=(hostVehicle, output_queue,read_lidar,emergecy_stop,data_ready, ref_alt))
    observer_thread.start()
    return observer_thread