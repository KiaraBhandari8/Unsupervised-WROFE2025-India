from time import sleep
import board
import busio
import adafruit_vl53l0x
import adafruit_tca9548a # Still import this, but we'll use PCA9546A from it

# --- I2C and Multiplexer Initialization ---
try:
    i2c = busio.I2C(board.SCL, board.SDA)
    # Initialize the PCA9546A object (it's part of the same library)
    tca = adafruit_tca9548a.PCA9546A(i2c)
except ValueError:
    print("I2C initialization failed. Check your connections.")
    exit()

# --- Sensor Initialization ---
# Create a sensor object for each sensor connected to the multiplexer.
# The PCA9546A has 4 channels: 0, 1, 2, 3.

# Initialize the first sensor on channel 0 (SD0, SC0)
try:
    sensor0 = adafruit_vl53l0x.VL53L0X(tca[0])
    print("Sensor 1 initialized on channel 0 (PCA9546A).")
except ValueError:
    print("Failed to initialize Sensor 1 on channel 0. Check wiring and power.")
    sensor0 = None

# Initialize the second sensor on channel 1 (SD1, SC1)
try:
    sensor1 = adafruit_vl53l0x.VL53L0X(tca[1])
    print("Sensor 2 initialized on channel 1 (PCA9546A).")
except ValueError:
    print("Failed to initialize Sensor 2 on channel 1. Check wiring and power.")
    sensor1 = None

# --- Reading from Both Sensors ---
print("\nStarting distance measurements...")
for i in range(20):
    if sensor0:
        # When you access sensor0, the library automatically tells the
        # PCA9546A to switch to channel 0.
        try:
            distance0 = sensor0.range
            print(f"Sensor 1 (Channel 0): {distance0} mm")
        except RuntimeError as e:
            print(f"Sensor 1 Error on Channel 0: {e}")

    if sensor1:
        # When you access sensor1, the library automatically switches
        # the PCA9546A to channel 1.
        try:
            distance1 = sensor1.range
            print(f"Sensor 2 (Channel 1): {distance1} mm")
        except RuntimeError as e:
            print(f"Sensor 2 Error on Channel 1: {e}")

    print("-" * 20)
    sleep(0.5)