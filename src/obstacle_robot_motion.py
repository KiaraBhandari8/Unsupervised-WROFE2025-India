from gpiozero.pins.lgpio import LGPIOFactory # Corrected capitalization
from gpiozero import DigitalOutputDevice, PWMOutputDevice, Device
from time import sleep

# --- IMPORTANT FOR RASPBERRY PI 5 ---
# Explicitly set the pin factory to lgpio
try:
    Device.pin_factory = LGPIOFactory()
    print("DEBUG: Using LGPIOFactory for gpiozero.")
except Exception as e:
    print(f"ERROR: Failed to set LGPIOFactory: {e}. Falling back to default.")
# --- END IMPORTANT ---

# Define GPIO pins for L298N motor driver
# These are BCM pin numbers
# Adjust these based on your specific wiring
AIN1 = DigitalOutputDevice(24) # Motor A Input 1 (GPIO24, Physical pin 18)
AIN2 = DigitalOutputDevice(23) # Motor A Input 2 (GPIO23, Physical pin 16)
BIN1 = DigitalOutputDevice(17) # Motor B Input 1 (GPIO17, Physical pin 11)
BIN2 = DigitalOutputDevice(27) # Motor B Input 2 (GPIO27, Physical pin 13)

# PWM pins for motor speed control (optional, if you have ENA/ENB connected)
# If your L298N has ENA/ENB pins and you want to control speed via PWM,
# uncomment these lines and connect them to appropriate PWM-capable GPIOs.
# ENA = PWMOutputDevice(12) # Motor A Enable (GPIO12, Physical pin 32)
# ENB = PWMOutputDevice(13) # Motor B Enable (GPIO13, Physical pin 33)

# Servo motor pin (usually PWM controlled)
# Using GPIO18 which is hardware PWM capable on many Raspberry Pis
SERVO_PIN = PWMOutputDevice(18, initial_value=0.075, frequency=50) # GPIO18, Physical pin 12

# Global variable for motor speed (0.0 to 1.0)
current_motor_speed = 0.9 # Default initial speed based on your main.py setting

def set_motor_speed(speed):
    """
    Sets the global motor speed for robot_forward(), robot_backward() etc.
    Speed should be between 0.0 (stop) and 1.0 (full speed).
    """
    global current_motor_speed
    if 0.0 <= speed <= 1.0:
        current_motor_speed = speed
        print(f"Robot: Motor speed set to {speed*100:.1f}%")
        # if 'ENA' in globals() and 'ENB' in globals():
        #    ENA.value = current_motor_speed
        #    ENB.value = current_motor_speed
    else:
        print("WARNING: Motor speed must be between 0.0 and 1.0. Speed not set.")

def robot_forward():
    """Moves the robot forward at the current_motor_speed."""
    # if 'ENA' in globals() and 'ENB' in globals():
    #    ENA.value = current_motor_speed
    #    ENB.value = current_motor_speed
    
    AIN1.on()
    AIN2.off()
    BIN1.on()
    BIN2.off()
    # print(f"Robot: Moving Forward at {current_motor_speed*100:.1f}%")

def robot_backward():
    """Moves the robot backward at the current_motor_speed."""
    # if 'ENA' in globals() and 'ENB' in globals():
    #    ENA.value = current_motor_speed
    #    ENB.value = current_motor_speed

    AIN1.off()
    AIN2.on()
    BIN1.off()
    BIN2.on()
    # print(f"Robot: Moving Backward at {current_motor_speed*100:.1f}%")

def robot_stop():
    """Stops the robot."""
    # if 'ENA' in globals() and 'ENB' in globals():
    #    ENA.off()
    #    ENB.off()

    AIN1.off()
    AIN2.off()
    BIN1.off()
    BIN2.off()
    # print("Robot: Stopping")

def robot_turn_left():
    """Turns the robot left by spinning wheels in opposite directions."""
    # if 'ENA' in globals() and 'ENB' in globals():
    #    ENA.value = current_motor_speed
    #    ENB.value = current_motor_speed
    
    AIN1.off() # Left motor backward
    AIN2.on()
    BIN1.on()  # Right motor forward
    BIN2.off()
    # print(f"Robot: Turning Left at {current_motor_speed*100:.1f}%")

def robot_turn_right():
    """Turns the robot right by spinning wheels in opposite directions."""
    # if 'ENA' in globals() and 'ENB' in globals():
    #    ENA.value = current_motor_speed
    #    ENB.value = current_motor_speed

    AIN1.on()  # Left motor forward
    AIN2.off()
    BIN1.off() # Right motor backward
    BIN2.on()
    # print(f"Robot: Turning Right at {current_motor_speed*100:.1f}%")

def adjust_servo_angle(angle):
    """
    Adjusts the servo angle based on a linear mapping.
    Assumes your servo's full range (0-180 degrees) corresponds to PWM duty cycles of ~0.02 to ~0.12.
    Your `main.py` uses angles like 45, 60, 75, 90, 105.
    This function interpolates those values to a suitable duty cycle.
    """
    # Mapping based on common servo ranges:
    # 45 degrees -> ~0.05 duty cycle
    # 75 degrees -> ~0.075 duty cycle (center)
    # 105 degrees -> ~0.10 duty cycle

    # Linear interpolation: y = y1 + (x - x1) * ((y2 - y1) / (x2 - x1))
    # Using points (45, 0.05) and (105, 0.10)
    duty_cycle = (angle - 45) * ((0.10 - 0.05) / (105 - 45)) + 0.05
    
    # Clamp duty cycle to ensure it's within safe operating range
    min_dc_clamp = 0.04 # Absolute minimum safe duty cycle for your servo
    max_dc_clamp = 0.11 # Absolute maximum safe duty cycle for your servo
    duty_cycle = max(min_dc_clamp, min(duty_cycle, max_dc_clamp))

    SERVO_PIN.value = duty_cycle
    # print(f"Servo moved to {angle} degrees. (Duty Cycle: {duty_cycle:.4f})") # Commented to reduce verbosity

def motor_standby():
    """
    Puts motors in a low-power, disengaged state.
    Ensures all motor control pins are off and any PWM enable pins are off.
    """
    robot_stop() # Ensure motors are stopped first
    # if 'ENA' in globals() and 'ENB' in globals():
    #    ENA.off()
    #    ENB.off()
    print("Motors: Standby Mode")


if __name__ == "__main__":
    # Example Usage (for testing this module independently)
    try:
        print("Testing obstacle_robot_motion.py...")
        set_motor_speed(0.6) # Set a test speed

        print("Moving forward for 2 seconds...")
        robot_forward()
        sleep(2)
        robot_stop()
        sleep(1)

        print("Adjusting servo to center (75 deg)...")
        adjust_servo_angle(75)
        sleep(1)
        print("Adjusting servo to left (90 deg)...")
        adjust_servo_angle(90)
        sleep(1)
        print("Adjusting servo to right (60 deg)...")
        adjust_servo_angle(60)
        sleep(1)
        print("Adjusting servo back to center (75 deg)...")
        adjust_servo_angle(75)
        sleep(1)

        print("Turning left for 1 second...")
        robot_turn_left()
        sleep(1)
        robot_stop()
        sleep(1)

        print("Turning right for 1 second...")
        robot_turn_right()
        sleep(1)
        robot_stop()
        sleep(1)

    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    finally:
        motor_standby()
        print("Test finished and motors are in standby.")