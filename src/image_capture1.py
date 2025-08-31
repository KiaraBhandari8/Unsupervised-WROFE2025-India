#!/usr/bin/env python3

import time
import os
import datetime
from picamera2 import Picamera2
from libcamera import Transform

print("Starting single image capture...")

# --- Configuration ---
HOME_DIR = os.path.expanduser("~")
OUTPUT_DIR = os.path.join(HOME_DIR, "wrofe2025/D:\Captured_images")

# --- Main Script ---
try:
    # Create the output directory if it doesn't exist
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(f"Image will be saved in: {OUTPUT_DIR}")

    # Initialize the camera
    picam2 = Picamera2()

    # --- SET YOUR DESIRED RESOLUTION HERE ---
    width = 1152
    height = 648

    print(f"Configuring camera for resolution: {width}x{height}")

    # Create a still configuration for the single image
    # We use a simple configuration without iterating through all sensor modes
    config = picam2.create_still_configuration(
        main={"size": (width, height)},
        transform=Transform(hflip=False, vflip=False)
    )
    
    picam2.configure(config)
    picam2.start()
    
    # Allow 2 seconds for auto-exposure and auto-focus to settle
    print("Allowing 2s for sensor to settle...")
    time.sleep(2)

    # Generate a unique filename with a timestamp
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"segment_{timestamp}.jpg"
    filepath = os.path.join(OUTPUT_DIR, filename)
    
    # Capture the image
    print(f"Capturing image to {filepath}...")
    picam2.capture_file(filepath)
    print("Capture complete.")

    # Stop the camera and clean up resources
    picam2.stop()
    picam2.close()
    
    print("\n-----------------------------")
    print("Image capture complete!")

except Exception as e:
    print(f"An error occurred: {e}")
    # Try to clean up by stopping the camera if it's running
    if 'picam2' in locals() and picam2.started:
        picam2.stop()
        print("Camera stopped due to error.")
    if 'picam2' in locals() and not picam2.closed:
        picam2.close()