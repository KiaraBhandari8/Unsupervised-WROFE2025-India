import time
import os # For creating directories
import threading # For running camera capture in a separate thread
import sys # For clean exit
import keyboard

from gpiozero import DigitalOutputDevice, PWMOutputDevice
from time import sleep # Already imported, keeping for clarity if used elsewhere

# Camera imports
from picamera2 import Picamera2
import libcamera
# Standard imports for Picamera2
# import libcamera.controls # Useful for advanced camera controls if needed

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

# --- Camera Setup ---
picam2 = None # Global variable to hold the Picamera2 object
camera_running_flag = False # Global flag to control the camera thread's loop

def camera_capture_thread():
    """
    This function runs in a separate thread to capture images periodically.
    """
    global picam2
    global camera_running_flag

    try:
        # Create a directory for images if it doesn't exist
        image_dir = "captured_images"
        os.makedirs(image_dir, exist_ok=True)
        print(f"Camera: Images will be saved to: {os.path.abspath(image_dir)}")

        picam2 = Picamera2()
        # Configure camera for a reasonable resolution for quick capture
        # main={"size": (1280, 720)} is a common choice. Adjust as needed.
        # Adding 'format' to explicitly set JPEG if desired, though .capture_file() implies it
        camera_config = picam2.create_preview_configuration(main={"size": (1640, 1232)},
                                                            transform=libcamera.Transform(vflip=True, hflip=True)) # <--- THIS IS THE CHANGE
        picam2.configure(camera_config)
        
        picam2.start()
        print("Camera: Started. Warming up...")
        # Give camera a moment to warm up and settle
        time.sleep(2) 
        print("Camera: Ready for capture.")

        camera_running_flag = True
        while camera_running_flag:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = os.path.join(image_dir, f"image_{timestamp}.jpg")
            
            # Capture a full resolution image (if using main stream)
            # You might need to specify a different stream for capture_file if configured
            # For simpler use, create_still_configuration() is often used for photos
            # but for periodic capture, the preview stream can work.
            picam2.capture_file(filename) 
            print(f"Camera: Captured: {filename}")
            time.sleep(0.5) # Capture every half second

    except Exception as e:
        print(f"Camera Error in capture thread: {e}")
    finally:
        if picam2:
            if picam2.started: # Check if camera was successfully started before stopping
                print("Camera: Stopping...")
                picam2.stop()
            picam2.close()
            print("Camera: Stopped and closed.")


# --- Keyboard Control ---
# Keep track of pressed keys for motor control to avoid stopping immediately
# and to manage "on_release" behavior
pressed_motor_keys = set()
exit_flag = False # Flag to signal main loop to exit

def handle_key_event(event):
    global pressed_motor_keys
    global current_servo_angle
    global exit_flag # Make sure exit_flag is global

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

        # Servo Control (using 'a'/'d' for incremental adjustment, 'z'/'c'/'x' for direct set)
        elif key_name == 'a' or key_name == 'left':
            adjust_servo_angle("increase") # Your code was set to 'increase' here
        elif key_name == 'd' or key_name == 'right':
            adjust_servo_angle("decrease") # Your code was set to 'decrease' here
        elif key_name == 'c': # Set servo to 50 degrees (from your latest code)
            current_servo_angle = 50
            kit.servo[SERVO_INDEX].angle = current_servo_angle
            print(f"Servo {SERVO_INDEX} set to {current_servo_angle} degrees (Key: C).")
        elif key_name == 'z': # Set servo to 100 degrees (from your latest code)
            current_servo_angle = 100
            kit.servo[SERVO_INDEX].angle = current_servo_angle
            print(f"Servo {SERVO_INDEX} set to {current_servo_angle} degrees (Key: Z).")
        elif key_name == 'x': # Set servo to 70 degrees (from your latest code)
            current_servo_angle = 70
            kit.servo[SERVO_INDEX].angle = current_servo_angle
            print(f"Servo {SERVO_INDEX} set to {current_servo_angle} degrees (Key: X).")
        elif key_name == 'esc':
            print("\nEscape pressed. Exiting...")
            exit_flag = True # Set the flag to terminate the main loop


    # Key release events
    elif event.event_type == keyboard.KEY_UP:
        key_name = event.name.lower()
        if key_name in pressed_motor_keys:
            pressed_motor_keys.remove(key_name)
            # Only stop if no other motor control keys are still pressed
            if not any(k in pressed_motor_keys for k in ['w', 's', 'up', 'down']):
                robot_stop()


# --- Main Program ---
if __name__ == "__main__":
    print("Robot Keyboard Control Started (using 'keyboard' library and Camera V2).")
    print("Use 'W' or Up Arrow for Forward, 'S' or Down Arrow for Backward.")
    print("Use 'A' or Left Arrow for Servo Angle INCREASE.")
    print("Use 'D' or Right Arrow for Servo Angle DECREASE.")
    print("Press 'Z' to set Servo to 100 degrees.")
    print("Press 'C' to set Servo to 50 degrees.")
    print("Press 'X' to set Servo to 70 degrees.")
    print("Press 'Space' to stop motor and reset servo.")
    print("Press 'Esc' to exit.")
    print("Ensure external motor power is ON and Camera V2 is connected/enabled.")
    print(f"Script last run: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())}")

    # Initialize GPIO (Motor & Servo)
    try:
        # GPIO setup for motors is implicitly handled by DigitalOutputDevice/PWMOutputDevice
        pass
    except Exception as e:
        print(f"Error during GPIO setup: {e}")
        sys.exit(1)

    camera_thread = None # Define thread variable here to ensure it's accessible in finally block
    try:
        # --- Start Camera Thread ---
        camera_thread = threading.Thread(target=camera_capture_thread)
        camera_thread.daemon = True # Allows the main program to exit even if this thread hasn't finished (though we'll explicitly stop it)
        camera_thread.start()
        # Give the camera thread a moment to initialize the camera
        time.sleep(3) # Increased sleep to ensure camera is fully ready before proceeding

        # --- Start Keyboard Listener ---
        keyboard.hook(handle_key_event)
        print("Keyboard listener started. Press keys to control robot.")

        # Keep the program running indefinitely until 'esc' is pressed (via exit_flag)
        while not exit_flag:
            time.sleep(0.1) # Small delay to reduce CPU usage

    except Exception as e:
        print(f"\nAn error occurred in main program: {e}")
        print("Ensure 'sudo' is used and 'keyboard' library can access input devices.")
    finally:
        print("Cleaning up resources...")
        # --- Stop Camera Thread Gracefully ---
        if camera_running_flag: # Check if the camera thread was ever fully started
            print("Signaling camera thread to stop...")
            camera_running_flag = False # Set flag to terminate camera thread's loop
            if camera_thread and camera_thread.is_alive():
                camera_thread.join(timeout=5) # Wait for thread to finish (max 5 seconds)
                if camera_thread.is_alive():
                    print("Camera thread did not terminate gracefully within timeout.")

        # --- Stop Keyboard Listener ---
        keyboard.unhook_all()
        print("Keyboard listener unhooked.")

        # --- Cleanup GPIO and Motors ---
        motor_standby()  # Ensure motor is off and in standby
        PWMA_ENABLE.close()
        AIN1.close()
        AIN2.close()
        STBY.close()
        print("GPIO cleanup complete. Exiting.")




# sudo /home/pi8/wrofe2025/env_test/bin/python /home/pi8/wrofe2025/robot_control_keys_1.py
# sudo /home/pi8/wrofe2025/env_test/bin/python /home/pi8/wrofe2025/rck_2.py