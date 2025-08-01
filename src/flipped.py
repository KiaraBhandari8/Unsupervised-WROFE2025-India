from picamera2 import Picamera2
from libcamera import Transform

# Initialize camera
picam2 = Picamera2()

# Create still configuration with size and flips set here
#config = picam2.create_still_configuration(main={"size": (768, 432)}, transform=Transform(vflip=True, hflip=True))
#config = picam2.create_still_configuration(main={"size": (4608, 2592)}, transform=Transform(vflip=True, hflip=True))
# config = picam2.create_still_configuration(main={"size": (2304, 1296)}, transform=Transform(vflip=True, hflip=True))
config = picam2.create_still_configuration(main={"size": (1152, 648)}, transform=Transform(vflip=True, hflip=True))

# Configure and start camera
picam2.configure(config)
picam2.start()

# Capture and save the image
picam2.capture_file("line_tracking8.jpg")

# Close the camera resource
picam2.close()

print("Image captured, flipped vertically and horizontally, and saved as 'line_tracking8.jpg'")
