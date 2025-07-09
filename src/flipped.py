from picamera2 import Picamera2
from libcamera import Transform

# Initialize camera
picam2 = Picamera2()

# Configure camera with both vertical and horizontal flip
config = picam2.create_still_configuration(transform=Transform(vflip=True, hflip=True))
picam2.configure(config)

# Start camera
picam2.start()
picam2.capture_file("flipped_current2.jpg")
picam2.close()

print("Image captured, flipped vertically and horizontally, and saved as 'flipped_image.jpg'")
