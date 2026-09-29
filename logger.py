import time

def log_data(data):
    with open("sensor_log.csv", "a") as file:
        file.write(
            str(time.ticks_ms()) + "," +
            str(data["temperature"]) + "," +
            str(data["pressure"]) + "," +
            str(data["ax"]) + "," +
            str(data["ay"]) + "," +
            str(data["az"]) + "," +
            str(data["gx"]) + "," +
            str(data["gy"]) + "," +
            str(data["gz"]) + "\n"
        )