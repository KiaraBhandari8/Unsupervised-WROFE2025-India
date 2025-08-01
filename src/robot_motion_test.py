import time
from gpiozero import DigitalOutputDevice, PWMOutputDevice
from time import sleep
import board
from adafruit_servokit import ServoKit
from adafruit_extended_bus import ExtendedI2C as I2C_Extended

# --- Motor Control Setup ---
# Define GPIO pins (BCM numbering)
AIN1 = DigitalOutputDevice(24)
AIN2 = DigitalOutputDevice(23)
PWMA_ENABLE = PWMOutputDevice(12)
STBY = DigitalOutputDevice(25)

MOTOR_SPEED = 0.45  # Adjust as needed

def motor_forward(speed=MOTOR_SPEED):
    STBY.on()
    AIN1.on()
    AIN2.off()
    PWMA_ENABLE.value = speed

def motor_backward(speed=MOTOR_SPEED):
    STBY.on()
    AIN1.off()
    AIN2.on()
    PWMA_ENABLE.value = speed

def motor_stop():
    STBY.on()
    AIN1.off()
    AIN2.off()
    PWMA_ENABLE.value = 0

def motor_brake():
    STBY.on()
    AIN1.on()
    AIN2.on()
    PWMA_ENABLE.value = 1

def motor_standby():
    PWMA_ENABLE.value = 0
    STBY.off()

def robot_forward():
    print("Robot: Moving Forward")
    motor_forward(MOTOR_SPEED)

def robot_backward():
    print("Robot: Moving Backward")
    motor_backward(MOTOR_SPEED)

def robot_stop():
    print("Robot: Stopping")
    motor_stop()

def robot_left_90():
    adjust_servo_angle(100)
    motor_forward(MOTOR_SPEED)
    sleep(2.5)

def robot_right_90():
    adjust_servo_angle(50)
    motor_forward(MOTOR_SPEED)
    sleep(2.5)

# --- Servo Setup ---
i2c_bus = I2C_Extended(4)
kit = ServoKit(channels=16, i2c=i2c_bus)

SERVO_INDEX = 0
SERVO_MIN_ANGLE = 50
SERVO_MAX_ANGLE = 100
current_servo_angle = 75

# Set PWM range
kit.servo[SERVO_INDEX].set_pulse_width_range(500, 2500)
kit.servo[SERVO_INDEX].angle = current_servo_angle
print(f"Servo {SERVO_INDEX} initialized to {current_servo_angle} degrees.")

def adjust_servo_angle(new_angle):
    clamped_angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, new_angle))
    kit.servo[SERVO_INDEX].angle = clamped_angle
    print(f"Servo {SERVO_INDEX} moved to {clamped_angle} degrees.")

if __name__ == "__main__":
    adjust_servo_angle(75)
    sleep(1)
    robot_left_90()
