import time
from gpiozero import DigitalOutputDevice, PWMOutputDevice
from time import sleep
import keyboard # Changed from pynput

import board
from adafruit_servokit import ServoKit
from adafruit_extended_bus import ExtendedI2C as I2C_Extended

# --- Motor Control Setup ---
# Define GPIO pins (BCM numbering)
# For Motor A
AIN1 = DigitalOutputDevice(24)  # Connected to TB6612FNG AIN1
AIN2 = DigitalOutputDevice(23)  # Connected to TB6612FNG AIN2
PWMA_ENABLE = PWMOutputDevice(12)  # Changed to PWMOutputDevice for speed control
STBY = DigitalOutputDevice(25)  # Connected to TB6612FNG STBY

MOTOR_SPEED = 1.0  # Default motor speed (can be adjusted)
# MOTOR_TURN_SPEED_DIFFERENCE = 0.3 # Not directly used with single motor, can remove if not expanding

def motor_forward(speed=MOTOR_SPEED):
    """
    Drives the motor forward at given speed (0.0 to 1.0).
    """
    STBY.on()
    AIN1.on()
    AIN2.off()
    PWMA_ENABLE.value = speed

def motor_backward(speed=MOTOR_SPEED):
    """
    Drives the motor backward at given speed (0.0 to 1.0).
    """
    STBY.on()
    AIN1.off()
    AIN2.on()
    PWMA_ENABLE.value = speed

def motor_stop():
    """
    Stops the motor (coasting).
    """
    STBY.on()
    AIN1.off()
    AIN2.off()
    PWMA_ENABLE.value = 0

def motor_brake():
    """
    Brakes the motor (short brake).
    PWMA must be high for brake to be effective.
    """
    STBY.on()
    AIN1.on()
    AIN2.on()
    PWMA_ENABLE.value = 1

def motor_standby():
    """
    Puts the motor driver in standby mode to save power.
    """
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


# --- Servo Control Setup ---
i2c_bus = I2C_Extended(4) # Use the bus ID (e.g., 4) that you set in config.txt
kit = ServoKit(channels=16, i2c=i2c_bus)

SERVO_INDEX = 0
SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 180
SERVO_INCREMENT = 5 # Degrees to move per key press

# Set initial servo position and pulse width range
kit.servo[SERVO_INDEX].set_pulse_width_range(500, 2500)
current_servo_angle = 75 # Initial angle
kit.servo[SERVO_INDEX].angle = current_servo_angle
print(f"Servo {SERVO_INDEX} initialized to {current_servo_angle} degrees.")

def adjust_servo_angle(direction):
    global current_servo_angle
    new_angle = current_servo_angle

    if direction == "increase":
        new_angle = min(SERVO_MAX_ANGLE, current_servo_angle + SERVO_INCREMENT)
    elif direction == "decrease":
        new_angle = max(SERVO_MIN_ANGLE, current_servo_angle - SERVO_INCREMENT)
    else:
        return

    if new_angle != current_servo_angle:
        current_servo_angle = new_angle
        kit.servo[SERVO_INDEX].angle = current_servo_angle
        print(f"Servo {SERVO_INDEX} moved to {current_servo_angle} degrees.")

# --- Keyboard Control ---

# Keep track of pressed keys for motor control to avoid stopping immediately
# and to manage "on_release" behavior
pressed_motor_keys = set()

def handle_key_event(event):
    global pressed_motor_keys
    # Declare current_servo_angle as global here, at the beginning of the function
    # where it is intended to be modified.
    global current_servo_angle # <--- MOVED THIS LINE UP

    # Key press events
    if event.event_type == keyboard.KEY_DOWN:
        key_name = event.name.lower() # Normalize key names

        # Motor Control
        if key_name == 'w' or key_name == 'up':
            if 'w' not in pressed_motor_keys and 'up' not in pressed_motor_keys:
                robot_forward()
            pressed_motor_keys.add(key_name)
        elif key_name == 's' or key_name == 'down':
            if 's' not in pressed_motor_keys and 'down' not in pressed_motor_keys:
                robot_backward()
            pressed_motor_keys.add(key_name)
        elif key_name == 'space':
            robot_stop()
            # Reset servo when space is pressed
            current_servo_angle = 75 # Initial angle
            kit.servo[SERVO_INDEX].angle = current_servo_angle
            print(f"Servo {SERVO_INDEX} reset to {current_servo_angle} degrees.")

        # Servo Control (using 'a'/'d' and arrow keys)
        elif key_name == 'a' or key_name == 'left':
            adjust_servo_angle("increase")
        elif key_name == 'd' or key_name == 'right':
            adjust_servo_angle("decrease")

        elif key_name == 'esc':
            print("\nEscape pressed. Exiting...")
            keyboard.unhook_all() # Stop listening to all keyboard events
            exit() # This is a hard exit, ensures program terminates quickly

    # Key release events
    elif event.event_type == keyboard.KEY_UP:
        key_name = event.name.lower()
        if key_name in pressed_motor_keys:
            pressed_motor_keys.remove(key_name)
            # Only stop if no other motor control keys are still pressed
            if not any(k in pressed_motor_keys for k in ['w', 's', 'up', 'down']):
                robot_stop()
                # Reset servo when all motor control keys are released


# --- Main Program ---
if __name__ == "__main__":
    print("Robot Keyboard Control Started (using 'keyboard' library).")
    print("Use 'W' or Up Arrow for Forward, 'S' or Down Arrow for Backward.")
    print("Use 'A' or Left Arrow for Servo Angle Decrease.")
    print("Use 'D' or Right Arrow for Servo Angle Increase.")
    print("Press 'Space' to stop motor and reset servo.")
    print("Press 'Esc' to exit.")
    print("Ensure external motor power is ON.")
    print(f"Script last run: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())}")


    try:
        # Hook into keyboard events
        keyboard.hook(handle_key_event)
        # Keep the program running indefinitely until 'esc' is pressed
        keyboard.wait('esc') # This will block until 'esc' is pressed
                               # The handle_key_event for 'esc' will call exit()

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Cleaning up GPIO and motors...")
        motor_standby()  # Ensure motor is off and in standby
        PWMA_ENABLE.close()
        AIN1.close()
        AIN2.close()
        STBY.close()
        print("GPIO cleanup complete. Exiting.")