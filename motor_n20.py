from gpiozero import DigitalOutputDevice, PWMOutputDevice
from time import sleep
import time

# Define GPIO pins (BCM numbering)
# For Motor A
AIN1 = DigitalOutputDevice(24)  # Connected to TB6612FNG AIN1
AIN2 = DigitalOutputDevice(23)  # Connected to TB6612FNG AIN2
PWMA_ENABLE = PWMOutputDevice(12)  # Changed to PWMOutputDevice for speed control
STBY = DigitalOutputDevice(25)  # Connected to TB6612FNG STBY

def motor_forward(speed=1.0):
    """
    Drives the motor forward at given speed (0.0 to 1.0).
    """
    STBY.on()          # Enable H-bridge
    AIN1.on()          # Set AIN1 HIGH
    AIN2.off()         # Set AIN2 LOW
    PWMA_ENABLE.value = speed  # Set speed via PWM duty cycle

def motor_backward(speed=1.0):
    """
    Drives the motor backward at given speed (0.0 to 1.0).
    """
    STBY.on()          # Enable H-bridge
    AIN1.off()         # Set AIN1 LOW
    AIN2.on()          # Set AIN2 HIGH
    PWMA_ENABLE.value = speed  # Set speed via PWM duty cycle

def motor_stop():
    """
    Stops the motor (coasting).
    """
    STBY.on()           # Keep H-bridge enabled for defined stop state
    AIN1.off()          # Set AIN1 LOW
    AIN2.off()          # Set AIN2 LOW
    PWMA_ENABLE.value = 0  # Disable motor output (PWM duty cycle 0)

def motor_brake():
    """
    Brakes the motor (short brake).
    PWMA must be high for brake to be effective.
    """
    STBY.on()           # Enable H-bridge
    AIN1.on()           # Set AIN1 HIGH
    AIN2.on()           # Set AIN2 HIGH (for brake on TB6612FNG)
    PWMA_ENABLE.value = 1  # Full PWM for effective brake

def motor_standby():
    """
    Puts the motor driver in standby mode to save power.
    """
    PWMA_ENABLE.value = 0  # Disable motor output first
    STBY.off()             # Then put driver in standby

# --- Main Program Example ---
if __name__ == "__main__":
    try:
        print("Motor control script with PWM speed control started.")
        print("Ensure external motor power is ON.")
        print(f"Script last run: {time.strftime('%Y-%m-%d %H:%M:%S')}")

        print("Driving motor forward at 50% speed for 5 seconds...")
        motor_forward(0.5)  # 50% speed
        sleep(5)

        print("Braking motor for 1 second...")
        motor_brake()
        sleep(1)

        print("Stopping motor (coast) for 1 second...")
        motor_stop()
        sleep(1)

        print("Putting motor driver in standby.")
        motor_standby()

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Cleaning up GPIO...")
        motor_standby()  # Ensure motor is off and in standby
        PWMA_ENABLE.close()
        AIN1.close()
        AIN2.close()
        STBY.close()
        print("GPIO cleanup complete. Exiting.")