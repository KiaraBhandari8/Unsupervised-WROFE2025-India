import time
import board
import busio
import adafruit_vl53l1x

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize VL53L1X sensor
vl53 = adafruit_vl53l1x.VL53L1X(i2c)

# Optionally set distance mode ("short" or "long")
vl53.distance_mode = 2  # 1 for short, 2 for long

# Start ranging
vl53.start_ranging()

try:
    while True:
        if vl53.data_ready:
            distance = vl53.distance
            print("Distance: {} cm".format(distance))
            vl53.clear_interrupt()
        time.sleep(0.2)
except KeyboardInterrupt:
    vl53.stop_ranging()
    print("Stopped.")