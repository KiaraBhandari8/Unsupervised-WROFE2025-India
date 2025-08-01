import cv2
import numpy as np
from picamera2 import Picamera2
import time
import libcamera
import matplotlib.pyplot as plt


picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(main={"size": (768, 432)},
                                                        transform=libcamera.Transform(vflip=True, hflip=True))
picam2.configure(camera_config)
picam2.start()
time.sleep(2)  # Camera warm-up

frame = picam2.capture_array()
if frame is None:
    print("Error: 'obs_frame.jpg' not found. Please ensure the image is in the correct path.")
    exit()

frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # Use original 'frame' (BGR) for HSV conversion

h, w, _ = frame.shape
print(f"Height : {h}, Width : {w}")

# --- Black Masking (Existing Code) ---
lower_black = np.array([0, 0, 0])
upper_black = np.array([180, 255, 130])
mask_black = cv2.inRange(hsv, lower_black, upper_black)

# --- Erosion ---
kernel = np.ones((5, 5), np.uint8)
mask_eroded = cv2.erode(mask_black, kernel, iterations=5)

# --- Find contours on eroded mask (black boundaries) ---
contours_black_boundaries, _ = cv2.findContours(mask_eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

if not contours_black_boundaries:
    print("No black boundaries found. Stopping.")
    print("STOP")
    exit()

# --- Keep only the two largest black boundary contours ---
contours_black_boundaries = sorted(contours_black_boundaries, key=cv2.contourArea, reverse=True)[:2]

# --- Create a mask for the selected black boundary components ---
largest_mask_boundaries = np.zeros_like(mask_eroded)
cv2.drawContours(largest_mask_boundaries, contours_black_boundaries, -1, 255, -1)

# --- Create the initial PATH MASK by inverting the largest_mask_boundaries ---
path_mask_initial = cv2.bitwise_not(largest_mask_boundaries)

# --- Filter path_mask_initial to keep only components with presence in the bottom 25% ---
bottom_25_percent_y = int(h * 0.75)
contours_path_components, _ = cv2.findContours(path_mask_initial, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

filtered_path_mask = np.zeros_like(path_mask_initial)
relevant_path_contour = None
max_area_in_bottom_25 = 0

for contour in contours_path_components:
    _, y, _, h_c = cv2.boundingRect(contour)
    if (y + h_c) >= bottom_25_percent_y:
        current_area = cv2.contourArea(contour)
        if current_area > max_area_in_bottom_25:
            max_area_in_bottom_25 = current_area
            relevant_path_contour = contour

if relevant_path_contour is not None:
    cv2.drawContours(filtered_path_mask, [relevant_path_contour], -1, 255, -1)
else:
    print("No significant path component found in the bottom 25% of the frame. Stopping.")
    print("STOP")
    exit()

current_drivable_path = filtered_path_mask

def filter_green_objects(hsv_frame):
    lower_green = np.array([30, 40, 40])
    upper_green = np.array([85, 255, 255])
    green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
    kernel_green = np.ones((5, 5), np.uint8)
    green_mask_eroded = cv2.erode(green_mask, kernel_green, iterations=3)
    green_mask_dilated = cv2.dilate(green_mask_eroded, kernel_green, iterations=1)
    green_signals, _ = cv2.findContours(green_mask_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return len(green_signals), green_signals, green_mask_dilated

def find_border_coordinates(contour, tolerance=2):
    if contour is None or len(contour) == 0:
        return None, None, None
    x_bbox, y_bbox, w_bbox, h_bbox = cv2.boundingRect(contour)

    left_border_points_x = []
    top_border_points_y = []

    # Iterate through all points in the contour
    for point in contour:
        px, py = point[0] # Contour points are typically in format [[[x,y]], [[x,y]], ...]
        if px <= x_bbox + tolerance:
            left_border_points_x.append(px)

        # Check if the point is on the topmost edge of the bounding box
        if py <= y_bbox + tolerance:
            top_border_points_y.append(py)

    # Calculate average coordinates
    avg_left_x = np.mean(left_border_points_x) if left_border_points_x else None
    avg_top_y = np.mean(top_border_points_y) if top_border_points_y else None

    return avg_left_x, avg_top_y, (x_bbox, y_bbox, w_bbox, h_bbox)

green_count, green_signals, green_mask_dilated = filter_green_objects(hsv)
largest_green_contour = max(green_signals, key=cv2.contourArea)
if green_count > 0:
    avg_left_x, avg_top_y, bbox = find_border_coordinates(largest_green_contour)
    y_coords, x_coords = np.indices(current_drivable_path.shape)
    region_to_zero_mask = (x_coords > avg_left_x) 
    current_drivable_path[region_to_zero_mask] = 0



plt.subplot(1,2,1)
plt.imshow(frame_rgb)
plt.title('Raw Frame')
plt.subplot(1,2,2)
plt.imshow(current_drivable_path, cmap='gray')
plt.title('Current Drivable Path')
plt.savefig('obstacle_result.jpg')

# You can save the mask if needed, as in your original code
# filename = "obs_out.jpg"
# cv2.imwrite(filename, green_mask_dilated)
# print(f"Green mask saved as {filename}")