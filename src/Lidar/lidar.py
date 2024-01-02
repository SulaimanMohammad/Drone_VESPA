import board
import adafruit_vl53l4cd
import time
import threading 
import queue
# Declare vl53 as a global variable
vl53 = None

def initialize_sensor():
    global vl53

    # Initialize I2C and sensor
    i2c = board.I2C()
    vl53 = adafruit_vl53l4cd.VL53L4CD(i2c)

    # Start ranging
    vl53.start_ranging()

def read_sensor(stop_event, queue):
    global vl53

    # Ensure the sensor is initialized
    if vl53 is None:
        raise Exception("Sensor not initialized")

    # Moving average parameters
    num_readings = 15
    readings = [0] * num_readings
    index = 0
    total = 0
    last_queued_distance = None
    last_update_time = time.time()
    update_interval = 0.5  # Update the queue every 0.5 second

    warm_up_period = 0.5  # Warm-up period in seconds
    start_time = time.time()

    while not stop_event.is_set():
        while not vl53.data_ready:
            pass
        vl53.clear_interrupt()

        # Remove the oldest reading and add the newest
        total -= readings[index]
        readings[index] = vl53.distance
        total += readings[index]

        index += 1
        if index >= num_readings:
            index = 0

        # Calculate the average
        average_distance = total / num_readings

        # Check if warm-up period has passed
        if time.time() - start_time > warm_up_period:
            # Check for significant change or if update interval has passed
            if (last_queued_distance is None or 
                abs(average_distance - last_queued_distance) >= 5 or 
                time.time() - last_update_time >= update_interval):
                print("Average Distance: {} cm".format(average_distance))
                queue.put(average_distance)
                last_queued_distance = average_distance
                last_update_time = time.time()



stop_event = threading.Event()
sensor_queue = queue.Queue()
thread = threading.Thread(target=read_sensor, args=(stop_event, sensor_queue))
thread.start()

initialize_sensor()

while True: 
    if not sensor_queue.empty():
        sensor_value = sensor_queue.get()

        # Check if sensor value is less than 80
        if sensor_value < 80:
            print("Sensor value below 80, entering special handling loop.")
            while sensor_value < 80:
                if not sensor_queue.empty():
                    sensor_value = sensor_queue.get()
                    print(f"GOing up, sensor value: {sensor_value}")
                time.sleep(0.1)  # Adjust as needed for responsiveness
            
        if sensor_value > 80:
            print("Sensor value above 80, exiting special handling loop.")
            print(f"GOing Down, sensor value: {sensor_value}")

