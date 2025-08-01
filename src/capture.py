import time
from picamera2 import Picamera2
# The Transform class is needed to apply flips
from libcamera import Transform

def capture_image_with_flip(filename="captured_image_flipped.jpg"):
    """
    Captures an image using the picamera2 library in a headless environment
    and applies a horizontal and vertical flip.
    """
    picam2 = Picamera2()

    # --- Configuration with Flipping ---
    # Create a configuration object. The `transform` parameter is used
    # to apply flips or rotations.
    # hflip=1 enables horizontal flip.
    # vflip=1 enables vertical flip.
    config = picam2.create_still_configuration(
        transform=Transform(hflip=1, vflip=1)
    )
    
    # Apply the configuration to the camera
    picam2.configure(config)

    print("Starting camera (headless mode with H/V flip)...")
    picam2.start() # This starts the camera capture pipeline

    # Give the camera a moment to adjust exposure and white balance.
    # This is crucial for good image quality.
    print("Camera warming up (2 seconds)...")
    time.sleep(2)

    print(f"Capturing image and saving to {filename}...")
    picam2.capture_file(filename)

    picam2.stop()
    print(f"Image captured successfully as {filename}!")

if __name__ == "__main__":
    capture_image_with_flip()
