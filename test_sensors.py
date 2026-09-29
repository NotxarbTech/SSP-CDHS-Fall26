import random
import time

def get_sensor_data():
    temperature = 70 + random.uniform(-1, 1)
    pressure = 1013 + random.uniform(-2, 2)
    
    ax = random.uniform(-0.1, 0.1)
    ay = random.uniform(-0.1, 0.1)
    az = 9.81 + random.uniform(-0.2, 0.2)
    
    gx = random.uniform(-2, 2)
    gy = random.uniform(-2, 2)
    gz = random.uniform(-2, 2)
    
    return {
        "temperature": temperature,
        "pressure": pressure,
        "ax": ax,
        "ay": ay,
        "az": az,
        "gx": gx,
        "gy": gy,
        "gz": gz,
    }