# Import the necessary libraries
# picamera2 is the modern library for the Raspberry Pi camera
# cv2 is OpenCV, used for drawing shapes on the image
import cv2
from picamera2 import Picamera2
import time

# --- Initialization ---
# Initialize the Picamera2 module
picam2 = Picamera2()

# Configure the camera for video capture.
# We set a resolution and format. 'BGR888' is compatible with OpenCV.
config = picam2.create_preview_configuration(main={"size": (1280, 720), "format": "BGR888"})
picam2.configure(config)

# Start the camera. The feed is now running.
picam2.start()

print("Camera feed started. Press 'q' in the preview window to exit.")

# --- Main Loop ---
# This loop continuously captures frames from the camera feed.
while True:
    # Capture a single frame from the camera as a NumPy array.
    frame = picam2.capture_array()

    # Get the dimensions of the frame (height, width, and channels).
    h, w, _ = frame.shape

    # --- Define Colors and Thickness ---
    # We'll draw the shapes in green. Color is in (B, G, R) format.
    color = (0, 255, 0)
    # The thickness of the lines for the shapes.
    thickness = 2

    # --- Calculate Coordinates ---
    # The coordinates are calculated based on the frame's width (w) and height (h)
    # as per your requirements. We convert them to integers as pixel coordinates
    # must be whole numbers.

    # 1. Outer Rectangle
    outer_rect_start = (int(0.25 * w), int(0.25 * h))
    outer_rect_end = (int(0.75 * w), int(0.75 * h))

    # 2. Inner Rectangle
    inner_rect_start = (int(0.25 * w), int(0.35 * h))
    inner_rect_end = (int(0.75 * w), int(0.65 * h))

    # 3. Vertical Line
    line_start = (int(0.5 * w), int(0.35 * h))
    line_end = (int(0.5 * w), int(0.65 * h))

    # --- Draw the Shapes on the Frame ---
    # cv2.rectangle() takes the frame, the top-left corner, the bottom-right corner,
    # the color, and the line thickness as arguments.

    # Draw the outer rectangle
    cv2.rectangle(frame, outer_rect_start, outer_rect_end, color, thickness)

    # Draw the inner rectangle
    cv2.rectangle(frame, inner_rect_start, inner_rect_end, color, thickness)

    # Draw the vertical line
    # cv2.line() takes the frame, start point, end point, color, and thickness.
    cv2.line(frame, line_start, line_end, color, thickness)

    # --- Display the Result ---
    # Show the modified frame in a window titled "Raspberry Pi Camera Feed".
    cv2.imshow("Raspberry Pi Camera Feed", frame)

    # --- Exit Condition ---
    # Wait for 1 millisecond for a key press.
    # If the key pressed is 'q', break out of the loop.
    # The `0xFF == ord('q')` part ensures this works across different systems.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
# When the loop is exited, stop the camera and close all OpenCV windows.
print("Stopping camera feed.")
picam2.stop()
cv2.destroyAllWindows()
