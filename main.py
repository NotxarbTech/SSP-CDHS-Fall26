import time

from bmp import BMP390
from mpu import MPU6050
from machine import I2C, Pin

import logger

# Driver initialization
i2c = I2C(0, sda=Pin(0), scl=Pin(1))
bmp = BMP390(i2c, address=0x77)

# mpu = MPU6050(sda=2, scl=3)

# Sleep time in seconds
SLEEP_TIME = 1

# Check if sensor log exists on the pico, if not make a new file with headers
try:
    with open("sensor_log.csv", "r"):
        pass
except:
    with open("sensor_log.csv", "w") as file:
        file.write("time_ms,temp,pressure,ax,ay,az,gx,gy,gz\n")

while True:
    # gyro_data = mpu.read_gyro_data()
    # accel_data = mpu.read_accel_data()
    gyro_data = {"x": "0",
                 "y": "0",
                 "z": "0",}
    accel_data = {"x": "0",
                  "y": "0",
                  "z": "0"}
    data = {
        "temperature": bmp.temperature,
        "pressure": bmp.pressure,
        "ax": accel_data["x"],
        "ay": accel_data["y"],
        "az": accel_data["z"],
        "gx": gyro_data["x"],
        "gy": gyro_data["y"],
        "gz": gyro_data["z"],
    }
    print(data)
    logger.log_data(data)
    time.sleep(SLEEP_TIME)