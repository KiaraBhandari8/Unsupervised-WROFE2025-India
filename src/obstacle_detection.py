import cv2
from picamera2 import Picamera2
import libcamera
import time, numpy as np

# Initialize Picamera2
picam2 = Picamera2()

# Create camera configuration
camera_config = picam2.create_preview_configuration(main={"size": (768, 432)},
                                                     transform=libcamera.Transform(vflip=True, hflip=True))

# Configure and start the camera
picam2.configure(camera_config)
picam2.start()

# Camera warm-up
time.sleep(2)

# Capture a frame as a NumPy array
frame = picam2.capture_array()
frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# Define the filename for saving the image
hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
lower_green = np.array([35,40, 40])
upper_green = np.array([86, 255, 255])
mask_green = cv2.inRange(hsv, lower_green, upper_green)

kernel3 = np.ones((3, 3), np.uint8)
kernel5 = np.ones((5, 5), np.uint8)
mask_eroded = cv2.erode(mask_green, kernel5, iterations=3)
mask_dilated = cv2.dilate(mask_eroded, kernel5, iterations=1)

filename = "obs_frame.jpg"

# Save the captured frame using OpenCV
cv2.imwrite(filename, frame)

# Stop the camera preview
picam2.stop()

print(f"Frame captured and saved as {filename}")