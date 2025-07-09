import time
import os
from picamera2 import Picamera2
import libcamera # Needed for libcamera.Transform if you use it

def test_camera_resolutions(output_dir="camera_test_images"):
    """
    Tests various camera resolutions and saves a sample image for each.
    This helps in understanding how different resolutions affect the Field of View.

    Args:
        output_dir (str): The directory where test images will be saved.
    """
    # Define a list of resolutions to test.
    # These are common resolutions, but you can add or remove others.
    # Note: The Camera Module 2's native sensor resolution is 3280x2464 (4:3 aspect ratio).
    # Resolutions that maintain this aspect ratio or are binned modes (like 1640x1232)
    # often provide the widest FOV. 16:9 resolutions (like 1920x1080) often crop the sensor.
    resolutions_to_test = [
        (640, 480),     # Standard VGA (4:3) - often a good FOV
        (1280, 720),    # HD (16:9) - common video resolution
        (1640, 1232),   # 2x2 binned mode (4:3) - often full FOV for V2 camera
        (1920, 1080),   # Full HD (16:9) - popular video resolution
        (3280, 2464)    # Full native sensor resolution (4:3) - highest detail, full FOV
    ]

    # Create the output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    print(f"Images will be saved to: {os.path.abspath(output_dir)}")

    picam2 = None # Initialize picam2 object outside the loop

    try:
        for res_width, res_height in resolutions_to_test:
            current_resolution = (res_width, res_height)
            print(f"\n--- Testing resolution: {current_resolution[0]}x{current_resolution[1]} ---")

            if picam2 is None:
                picam2 = Picamera2()

            # Configure the camera.
            # We use create_still_configuration for high-quality still captures.
            # The 'main' stream is where the image data will come from.
            camera_config = picam2.create_still_configuration(
                main={"size": current_resolution},
                # You can add a transform if your camera is mounted upside down etc.
                transform=libcamera.Transform(vflip=True, hflip=True)
            )
            picam2.configure(camera_config)

            # Start the camera
            picam2.start()
            print("Camera started. Warming up...")
            time.sleep(2) # Give the camera time to warm up and settle

            # Construct filename
            filename = os.path.join(output_dir, f"image_{res_width}x{res_height}.jpg")

            # Capture the image
            picam2.capture_file(filename)
            print(f"Captured image: {filename}")

            # Stop and close the camera for the next configuration
            picam2.stop()
            picam2.close()
            picam2 = None # Reset picam2 so it's re-initialized for the next resolution
            time.sleep(1) # Small delay before the next test

    except Exception as e:
        print(f"\nAn error occurred: {e}")
        print("Please ensure the camera is connected, enabled, and you have necessary permissions (e.g., run with 'sudo').")
    finally:
        # Ensure camera is stopped and closed if an error occurred or loop finished
        if picam2:
            if picam2.started:
                picam2.stop()
            picam2.close()
        print("\nCamera resolution testing complete.")

if __name__ == "__main__":
    test_camera_resolutions()
