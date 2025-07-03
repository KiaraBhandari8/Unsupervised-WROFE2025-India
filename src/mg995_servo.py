import board
from adafruit_servokit import ServoKit
import time

from adafruit_extended_bus import ExtendedI2C as I2C_Extended

i2c_bus = I2C_Extended(4) # Use the bus ID (e.g., 4) that you set in config.txt
kit = ServoKit(channels=16, i2c=i2c_bus)

kit.servo[0].set_pulse_width_range(500, 2500)
# kit.servo[0].angle = 81
# time.sleep(5)

for angle in range(70, 100, 3):
    kit.servo[0].angle = angle
    print(f"Servo Angle : {angle}")
    time.sleep(1)

for angle in range(100, 10, -3):
    kit.servo[0].angle = angle
    print(f"Servo Angle : {angle}")
    time.sleep(1)

# kit.servo[0].angle = 63