import time
from bmp import *
from mpu import MPU6050
from machine import I2C, Pin
import test_sensors
import logger

bmp_bus = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)
bmp = BMP280(bmp_bus, addr=0x77)

mpu = MPU6050(sda=2, scl=3)

print(bmp.temperature)
print(bmp.pressure)

print(mpu.read_accel_data())
print(mpu.read_temperature())

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