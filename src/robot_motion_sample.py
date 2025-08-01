# robot_motion_sample.py

import RPi.GPIO as GPIO
from adafruit_servokit import ServoKit
import time
from adafruit_extended_bus import ExtendedI2C as I2C_Extended



# Motor GPIO pin assignments
IN1 = 17
IN2 = 18
IN3 = 22
IN4 = 23
ENA = 24  # Left PWM
ENB = 25  # Right PWM

# PWM objects
pwm_left = None
pwm_right = None

# Optional ServoKit
try:
    i2c_bus = I2C_Extended(4) # Use the bus ID (e.g., 4) that you set in config.txt
    kit = ServoKit(channels=16, i2c=i2c_bus)
    kit.servo[0].angle = 75
    servo_available = True
    print("✅ ServoKit initialized.")
except Exception as e:
    print(f"⚠️ ServoKit init failed: {e}")
    kit = None
    servo_available = False

def setup():
    global pwm_left, pwm_right
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)
    pwm_left = GPIO.PWM(ENA, 100)
    pwm_right = GPIO.PWM(ENB, 100)
    pwm_left.start(0)
    pwm_right.start(0)

def set_motor_speed(percent):
    global pwm_left, pwm_right
    duty = max(0, min(100, percent * 100))
    pwm_left.ChangeDutyCycle(duty)
    pwm_right.ChangeDutyCycle(duty)

def robot_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(90)
    pwm_right.ChangeDutyCycle(90)
    print("Robot: Moving Forward at 90%")

def robot_stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)
    print("Robot: STOPPED")

def adjust_servo_angle(angle):
    if servo_available:
        try:
            kit.servo[0].angle = angle
            print(f"Servo 0 moved to {angle:.2f} degrees.")
        except Exception as e:
            print(f"Servo set failed: {e}")
    else:
        print(f"⚠️ Servo unavailable: desired angle {angle:.2f}° ignored.")

def motor_standby():
    GPIO.cleanup()
