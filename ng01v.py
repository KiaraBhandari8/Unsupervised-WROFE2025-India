from time import sleep
import board
import busio
import adafruit_vl53l0x
import adafruit_tca9548a

try:
    i2c = busio.I2C(board.SCL, board.SDA)
    tca = adafruit_tca9548a.TCA9548A(i2c)
except ValueError:
    print("I2C initialization failed. Check your connections.")
    exit()

try:
    sensor1 = adafruit_vl53l0x.VL53L0X(tca[0])
    print("Sensor 1 initialized on channel 0.")
except ValueError:
    print("Failed to initialize Sensor 1 on channel 0. Check wiring and power.")
    sensor1 = None

# Initialize the first sensor on channel 0 (SD0, SC0)
try:
    sensor2 = adafruit_vl53l0x.VL53L0X(tca[1])
    print("Sensor 2 initialized on channel 1.")
except ValueError:
    print("Failed to initialize Sensor 2 on channel 1. Check wiring and power.")
    sensor2 = None

# try:
#     sensor3 = adafruit_vl53l0x.VL53L0X(tca[4])
#     print("Sensor 3 initialized on channel 4.")
# except ValueError:
#     print("Failed to initialize Sensor 3 on channel 4. Check wiring and power.")
#     sensor3 = None


print("\nStarting distance measurements...")
for i in range(1000):
    if sensor1:
        try:
            distance1 = sensor1.range
            print(f"Sensor 1 (Channel 0): {distance1//10} cm")
        except RuntimeError as e:
            print(f"Sensor 1 Error: {e}")

    if sensor2:
        try:
            distance2 = sensor2.range
            print(f"Sensor 2 (Channel 1): {distance2//10} cm")
        except RuntimeError as e:
            print(f"Sensor 2 Error: {e}")
    
    # if sensor3:
    #     try:
    #         distance3 = sensor3.range
    #         print(f"Sensor 3 (Channel 4): {distance3//10} cm")
    #     except RuntimeError as e:
    #         print(f"Sensor 3 Error: {e}")

    print("-" * 20)
    sleep(0.5)