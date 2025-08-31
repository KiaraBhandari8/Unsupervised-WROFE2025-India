from picamera2 import Picamera2
from libcamera import Transform
import datetime
import os

picam2 = Picamera2()

config = picam2.create_still_configuration(main={"size": (1152, 648)}, transform=Transform(vflip=True, hflip=True))

picam2.configure(config)
picam2.start()

save_folder = "C:\\Users\\saksh\\OneDrive\\Desktop\\Capture_Image_ColorDetection"

os.makedirs(save_folder, exist_ok=True)

timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
image_filename = f"line_tracking_{timestamp}.jpg"

full_path = os.path.join(save_folder, image_filename)

picam2.capture_file(full_path)

picam2.close()

print(f"Image captured and saved to '{full_path}'")