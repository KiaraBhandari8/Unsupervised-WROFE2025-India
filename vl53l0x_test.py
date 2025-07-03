import board
import busio
import adafruit_vl53l0x
import time

i2c = busio.I2C(board.SCL, board.SDA)

vl53 = adafruit_vl53l0x.VL53L0X(i2c)
vl53.measurement_timing_budget = 200000  # Set to 200ms for high accuracy

CALIBRATED_OFFSET_CM = 3.5
print("VL53L0X Simple Distance Measurement Test")

try:
    while True:
        distance_mm = vl53.range

        if distance_mm is not None:
            actual_distance_cm = (distance_mm / 10.0) - CALIBRATED_OFFSET_CM
            if actual_distance_cm < 0:
                actual_distance_cm = 0.0
            # print(f"Distance: {distance_mm / 10.0:.2f} cm")
            print(f"Raw Dist: {distance_mm/10.0:.2f} cm, Corrected Dist: {actual_distance_cm:.2f} cm")
        else:
            print("Out of range or error occurred.")
        time.sleep(0.1) 

except KeyboardInterrupt:
    print("\nExiting distance measurement.")
