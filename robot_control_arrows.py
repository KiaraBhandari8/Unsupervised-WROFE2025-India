import time
from gpiozero import DigitalOutputDevice, PWMOutputDevice
from time import sleep
from pynput import keyboard

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

MOTOR_SPEED = 0.5  # Default motor speed (can be adjusted)
MOTOR_TURN_SPEED_DIFFERENCE = 0.3 # Difference in speed for turning

def motor_forward(speed=MOTOR_SPEED):
    """
    Drives the motor forward at given speed (0.0 to 1.0).
    """
    STBY.on()            # Enable H-bridge
    AIN1.on()            # Set AIN1 HIGH
    AIN2.off()           # Set AIN2 LOW
    PWMA_ENABLE.value = speed  # Set speed via PWM duty cycle

def motor_backward(speed=MOTOR_SPEED):
    """
    Drives the motor backward at given speed (0.0 to 1.0).
    """
    STBY.on()            # Enable H-bridge
    AIN1.off()           # Set AIN1 LOW
    AIN2.on()            # Set AIN2 HIGH
    PWMA_ENABLE.value = speed  # Set speed via PWM duty cycle

def motor_stop():
    """
    Stops the motor (coasting).
    """
    STBY.on()            # Keep H-bridge enabled for defined stop state
    AIN1.off()           # Set AIN1 LOW
    AIN2.off()           # Set AIN2 LOW
    PWMA_ENABLE.value = 0  # Disable motor output (PWM duty cycle 0)

def motor_brake():
    """
    Brakes the motor (short brake).
    PWMA must be high for brake to be effective.
    """
    STBY.on()             # Enable H-bridge
    AIN1.on()             # Set AIN1 HIGH
    AIN2.on()             # Set AIN2 HIGH (for brake on TB6612FNG)
    PWMA_ENABLE.value = 1  # Full PWM for effective brake

def motor_standby():
    """
    Puts the motor driver in standby mode to save power.
    """
    PWMA_ENABLE.value = 0   # Disable motor output first
    STBY.off()              # Then put driver in standby

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
def on_press(key):
    try:
        # Motor Control
        if key == keyboard.KeyCode.from_char('w') or key == keyboard.Key.up:
            robot_forward()
        elif key == keyboard.KeyCode.from_char('s') or key == keyboard.Key.down:
            robot_backward()
        elif key == keyboard.KeyCode.from_char('a') or key == keyboard.Key.left:
            adjust_servo_angle("decrease")
        elif key == keyboard.KeyCode.from_char('d') or key == keyboard.Key.right:
            adjust_servo_angle("increase")
        elif key == keyboard.Key.space:
            robot_stop()
            current_servo_angle = 75 # Initial angle
            kit.servo[SERVO_INDEX].angle = current_servo_angle
            print(f"Servo {SERVO_INDEX} reset to {current_servo_angle} degrees.")

        elif key == keyboard.Key.esc:
            # Stop listener
            return False
    except AttributeError:
        # Handle special keys (like arrow keys) which don't have .char attribute
        # These are handled by the 'key == keyboard.Key.left' etc. checks
        pass

def on_release(key):
    try:
        # Stop motors when 'w' or 's' keys are released
        # Exclude 'a' and 'd' from motor stop, as they now control the servo
        if key == keyboard.KeyCode.from_char('w') or \
           key == keyboard.KeyCode.from_char('s'):
            # After w or s is released then set the servo to initial angle.
            current_servo_angle = 75 # Initial angle
            kit.servo[SERVO_INDEX].angle = current_servo_angle
            print(f"Servo {SERVO_INDEX} reset to {current_servo_angle} degrees.")
    except AttributeError:
        pass


# --- Main Program ---
if __name__ == "__main__":
    print("Robot Keyboard Control Started.")
    print("Use 'W' for Forward, 'S' for Backward.")
    print("Use 'A' or Left Arrow for Servo Angle Decrease.")
    print("Use 'D' or Right Arrow for Servo Angle Increase.")
    print("Press 'Esc' to exit.")
    print("Ensure external motor power is ON.")
    print(f"Script last run: {time.strftime('%Y-%m-%d %H:%M:%S')}")

    # Set up keyboard listener
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        # Keep the main thread alive while the listener runs in the background
        listener.join()
    except KeyboardInterrupt:
        print("\nProgram stopped by user (Ctrl+C).")
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