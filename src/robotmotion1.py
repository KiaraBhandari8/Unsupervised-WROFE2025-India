from gpiozero import DigitalOutputDevice, PWMOutputDevice
from adafruit_servokit import ServoKit
from adafruit_extended_bus import ExtendedI2C as I2C_Extended

# Motor Control Pins (BCM numbering) as per your original code
AIN1 = DigitalOutputDevice(24)  # TB6612FNG AIN1
AIN2 = DigitalOutputDevice(23)  # TB6612FNG AIN2
PWMA_ENABLE = PWMOutputDevice(12)  # PWM for speed control on PWMA pin
STBY = DigitalOutputDevice(25)  # Standby pin

MOTOR_SPEED = 0.5  # Adjust as needed (0.0 to 1.0)

def drive_forward():
    STBY.on()
    AIN1.on()
    AIN2.off()
    PWMA_ENABLE.value = MOTOR_SPEED

def stop():
    STBY.on()
    AIN1.off()
    AIN2.off()
    PWMA_ENABLE.value = 0

# Servo setup using your I2C bus (bus 4 as in your code)
i2c_bus = I2C_Extended(4)
kit = ServoKit(channels=16, i2c=i2c_bus)

SERVO_INDEX = 0

# Initialize servo with pulse width range as in your original code
kit.servo[SERVO_INDEX].set_pulse_width_range(500, 2500)
# Optional: set initial servo angle to 75 degrees (center)
kit.servo[SERVO_INDEX].angle = 75

def set_steering(angle):
    # Clamp angle between 0 and 180
    angle = max(0, min(180, angle))
    kit.servo[SERVO_INDEX].angle = angle
