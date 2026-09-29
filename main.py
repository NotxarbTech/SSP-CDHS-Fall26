import time

import test_sensors
import logger

# Check if sensor log exists on the pico, if not make a new file with headers
try:
    with open("sensor_log.csv", "r"):
        pass
except:
    with open("sensor_log.csv", "w") as file:
        file.write("time_ms,temp,pressure,ax,ay,az,gx,gy,gz\n")

while True:
    logger.log_data(test_sensors.get_sensor_data())
    time.sleep(1)