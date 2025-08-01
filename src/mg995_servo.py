import board
from adafruit_servokit import ServoKit
import time

from adafruit_extended_bus import ExtendedI2C as I2C_Extended

i2c_bus = I2C_Extended(4) # Use the bus ID (e.g., 4) that you set in config.txt
kit = ServoKit(channels=16, i2c=i2c_bus)

kit.servo[1].set_pulse_width_range(500, 2500)
# kit.servo[1].angle = 90
# time.sleep(0.25)
# kit.servo[1].angle = 70
# time.sleep(0.25)
kit.servo[1].angle = 90
time.sleep(0.25)
# kit.servo[1].angle = 110
# time.sleep(0.25)
# kit.servo[1].angle = 90
# time.sleep(0.25)



# time.sleep(5)

for angle in range(90, 65, -3):
    kit.servo[1].angle = angle
    print(f"Servo Angle : {angle}")
    time.sleep(0.5)

# kit.servo[1].angle = 90
# for angle in range(100, 70, -3):
#     kit.servo[0].angle = angle
#     print(f"Servo Angle : {angle}")
#     time.sleep(1)

# kit.servo[0].angle = 75