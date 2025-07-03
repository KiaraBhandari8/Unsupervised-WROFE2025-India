import time
from gpiozero import DigitalOutputDevice
import keyboard # Using the 'keyboard' library
import sys # For clean exit

# Define the GPIO pin for the LED (BCM numbering)
LED_PIN = 26
led = None # Initialize led to None for proper cleanup

def setup_gpio():
    """Initializes the GPIO device for the LED."""
    global led
    try:
        led = DigitalOutputDevice(LED_PIN)
        print(f"LED initialized on GPIO BCM {LED_PIN}.")
        led.off() # Ensure LED is off at start
    except Exception as e:
        print(f"Error setting up GPIO: {e}")
        print("Ensure the pin number is correct and gpiozero is installed.")
        sys.exit(1) # Exit if GPIO setup fails

# Flag to signal the main loop to exit cleanly
exit_flag = False

# This set is useful for handling continuous presses in motor control,
# but for simple LED ON/OFF, its primary use here is to ensure the action
# only triggers once per press, not on auto-repeat.
pressed_keys_tracking = set()

def handle_key_event(event):
    """
    Handles both key press (KEY_DOWN) and key release (KEY_UP) events.
    This function will be called by the 'keyboard' library for every event.
    """
    global led
    global exit_flag
    global pressed_keys_tracking # Declare global for the tracking set

    # Only process key down events for this LED control
    if event.event_type == keyboard.KEY_DOWN:
        key_name = event.name.lower() # Normalize key names to lowercase

        # Prevent action on key auto-repeat (only act on the initial press)
        if key_name in pressed_keys_tracking:
            return # Key is already being held down

        pressed_keys_tracking.add(key_name) # Mark key as currently pressed

        if key_name == 'a':
            if led:
                led.on()
                print("LED ON (Key: A)")
            else:
                print("LED object not initialized. Cannot turn ON.")
        elif key_name == 'd':
            if led:
                led.off()
                print("LED OFF (Key: D)")
            else:
                print("LED object not initialized. Cannot turn OFF.")
        elif key_name == 'esc':
            print("\nEscape pressed. Exiting...")
            exit_flag = True # Set flag to terminate the main loop

    # Handle key release events to remove from tracking set
    elif event.event_type == keyboard.KEY_UP:
        key_name = event.name.lower()
        if key_name in pressed_keys_tracking:
            pressed_keys_tracking.remove(key_name)


# --- Main Program ---
if __name__ == "__main__":
    print("--- LED Control with 'keyboard' library ---")
    print(f"Controlling LED on GPIO BCM {LED_PIN}.")
    print("Press 'A' to turn LED ON.")
    print("Press 'D' to turn LED OFF.")
    print("Press 'Esc' to exit.")
    print("\nIMPORTANT: This script must be run with 'sudo' on Linux.")
    print(f"Script last run: {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())}")

    setup_gpio() # Initialize the LED GPIO

    try:
        # Hook into keyboard events
        keyboard.hook(handle_key_event)

        # Keep the program running indefinitely until 'esc' is pressed
        # Use a loop with a small sleep and check the exit_flag
        while not exit_flag:
            time.sleep(0.1) # Small delay to reduce CPU usage

    except Exception as e:
        print(f"\nAn error occurred: {e}")
        print("Possible causes: Permissions (did you run with sudo?), or environment not suitable for 'keyboard' library.")
    finally:
        print("Cleaning up GPIO...")
        # Unhook keyboard listener before GPIO cleanup
        keyboard.unhook_all()
        if led:
            led.off() # Ensure LED is off before closing
            led.close() # Release the GPIO pin
        print("GPIO cleanup complete. Exiting.")