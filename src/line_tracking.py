import time
import cv2
import numpy as np
from picamera2 import Picamera2
from libcamera import Transform

def filter_blue_objects(hsv_frame):
    """
    Processes an HSV image to find and count distinct blue objects.
    """
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([130, 255, 255])
    
    blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
    
    kernel = np.ones((5, 5), np.uint8)
    blue_mask_eroded = cv2.erode(blue_mask, kernel, iterations=2)
    blue_mask_dilated = cv2.dilate(blue_mask_eroded, kernel, iterations=2)
    
    blue_contours, _ = cv2.findContours(blue_mask_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    return len(blue_contours), blue_contours, blue_mask_dilated


def main_loop():
    """
    Initializes the camera and runs the main detection loop.
    """
    # --- One-Time Initialization ---
    picam2 = Picamera2()
    config = picam2.create_still_configuration(
        # Apply a horizontal and vertical flip if your camera is mounted upside down
        transform=Transform(hflip=1, vflip=1)
    )
    picam2.configure(config)
    picam2.start()
    print("Camera started. Starting detection loop...")
    print("Press Ctrl+C to exit.")

    # --- State and Counter Variables ---
    # Initialize these *before* the loop starts.
    line_counter = 0
    previous_blue_state = False # Assume we don't see blue at the start

    try:
        while True:
            # 1. Capture a frame
            frame = picam2.capture_array()

            # 2. Convert to HSV color space for processing
            # Note: picamera2 captures in RGB, so we convert RGB to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

            # 3. Get the count of blue objects
            blue_count, _, _ = filter_blue_objects(hsv)

            # 4. Determine the current state
            current_blue_state = (blue_count > 0)

            # --- Counter Logic ---
            # Check for the state change: from SEEING blue to NOT seeing blue.
            # This is the moment the robot has finished passing the line.
            if not current_blue_state and previous_blue_state:
                line_counter += 1
                print(f"Blue line passed! Counter: {line_counter}")

            # 5. Update the previous state for the next loop iteration
            previous_blue_state = current_blue_state
            
            # Optional: a small delay to prevent overwhelming the CPU
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nProgram stopped by user.")
    finally:
        # Ensure the camera is stopped cleanly on exit
        picam2.stop()
        print("Camera stopped.")


if __name__ == "__main__":
    main_loop()