import board
import adafruit_vl53l4cd
import busio

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
    num_readings = 10
    readings = [0] * num_readings  # List to store the last 'num_readings' values
    index = 0
    total = 0

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

        # Calculate the average and put it in the queue
        average_distance = total / num_readings
        print("Average Distance: {} cm".format(average_distance))
        queue.put(average_distance)