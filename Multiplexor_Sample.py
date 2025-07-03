from time import sleep
import board
import busio
import adafruit_vl53l0x
import adafruit_tca9548a   # for multiplexer control

i2c = busio.I2C(board.SCL, board.SDA)
tca = adafruit_tca9548a.TCA9548A(i2c)

# sensor1 = adafruit_vl53l0x.VL53L0X(tca[0])
# for i in range(10):
#     print(sensor1.distance)
#     sleep(0.25)

try:
    sensor1 = adafruit_vl53l0x.VL53L0X(tca[0])
    print("Sensor 1 initialized on channel 0.")
except ValueError:
    print("Failed to initialize Sensor 1 on channel 0. Check wiring and power.")
    sensor1 = None

# Initialize the second sensor on channel 1 (SD1, SC1)
try:
    sensor2 = adafruit_vl53l0x.VL53L0X(tca[1])
    print("Sensor 2 initialized on channel 1.")
except ValueError:
    print("Failed to initialize Sensor 2 on channel 1. Check wiring and power.")
    sensor2 = None
