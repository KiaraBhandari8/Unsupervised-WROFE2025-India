import keyboard
import time
import sys # For sys.exit()

def on_key_event(event):
    """
    Callback function that gets executed whenever a keyboard event occurs.
    """
    if event.event_type == keyboard.KEY_DOWN:
        key_name = event.name
        print(f"Key Pressed: {key_name}")
        # If 'esc' is pressed, stop listening and exit
        if key_name == 'esc':
            print("Escape key pressed. Stopping listener and exiting.")
            keyboard.unhook_all() # Stop all keyboard listeners
            sys.exit(0) # Exit the script

if __name__ == "__main__":
    print("--- Keyboard Input Test ---")
    print("Press any key to see its name.")
    print("Press 'Esc' to exit.")

    try:
        # Hook the function to all keyboard events
        keyboard.hook(on_key_event)
        # Keep the script running indefinitely
        # A simple time.sleep(1) loop is often better than keyboard.wait()
        # for debugging, as it allows other threads/processes to run.
        # Alternatively, you can use keyboard.wait('esc') to block until 'esc' is pressed.
        # For this test, let's just loop and print a heartbeat.
        while True:
            print(f"Still listening... {time.time()}")
            time.sleep(5) # Print every 5 seconds to show it's alive

    except PermissionError:
        print("\nERROR: Permission denied. You must run this script with sudo on Linux.")
        print("Try: sudo python3 your_script_name.py")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        print("Cleaning up keyboard listener.")
        keyboard.unhook_all() # Ensure cleanup even on unexpected exit
        print("Exited.")