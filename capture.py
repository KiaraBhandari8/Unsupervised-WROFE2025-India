import time
from picamera2 import Picamera2

def capture_image_picamera2_headless(filename="captured_image_picamera2_headless.jpg"):
    """
    Captures an image using the picamera2 library in a headless environment.
    No preview window will be displayed.
    """
    picam2 = Picamera2()

    # --- Configuration (Optional but Recommended) ---
    # For a still image, you often want the highest resolution available.
    # picam2.create_still_configuration() is usually a good default.
    # You can customize it if needed, e.g., for a specific resolution:
    # config = picam2.create_still_configuration(main={"size": (1920, 1080)})
    # picam2.configure(config)
    
    # Alternatively, for simplicity, just start the camera without specific configuration
    # if you're happy with its default still image settings.
    # picam2.start() will internally configure for still capture.

    print("Starting camera (headless mode)...")
    picam2.start() # This starts the camera capture pipeline without a preview

    # Give the camera a moment to adjust exposure and white balance
    # This is crucial for good image quality, especially in varying light.
    print("Camera warming up (2 seconds)...")
    time.sleep(2)

    print(f"Capturing image and saving to {filename}...")
    picam2.capture_file(filename)

    picam2.stop()
    print(f"Image captured successfully as {filename}!")

if __name__ == "__main__":
    capture_image_picamera2_headless()