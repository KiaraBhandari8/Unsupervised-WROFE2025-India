import cv2
import numpy as np
import math

# A simple class to hold data about the most relevant pillar
class Pillar:
    def __init__(self):
        self.distance = float('inf')
        self.area = 0
        self.x = 0
        self.y = 0
        self.color = None  # 'red' or 'green'

def analyze_scene(frame, rois):
    """
    Analyzes a single camera frame using the provided Regions of Interest (ROIs).
    This function does NOT make decisions; it only reports what it sees within the given rectangles.
    
    Args:
        frame: The camera image to analyze.
        rois (dict): A dictionary containing the ROI tuples for each object type.
    
    Returns:
        - A processed frame with debug drawings.
        - A dictionary containing all the analyzed data.
    """
    if frame is None:
        return None, {}

    # --- Initial Setup ---
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = frame.shape[:2]
    processed_frame = frame.copy()

    # --- Color Definitions ---
    lower_green = np.array([35, 100, 50])
    upper_green = np.array([85, 255, 255])
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([140, 255, 255])
    lower_orange = np.array([10, 150, 150])
    upper_orange = np.array([25, 255, 255])
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 60])

    # --- Create Masks ---
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_red = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), cv2.inRange(hsv, lower_red2, upper_red2))
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
    mask_black = cv2.inRange(hsv, lower_black, upper_black)

    # --- Analyze Objects in ROIs passed from the main script ---

    # 1. Find Wall Areas for Lane Keeping
    wl_x1, wl_y1, wl_x2, wl_y2 = rois['wall_left']
    wr_x1, wr_y1, wr_x2, wr_y2 = rois['wall_right']
    left_wall_contours, _ = cv2.findContours(mask_black[wl_y1:wl_y2, wl_x1:wl_x2], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    right_wall_contours, _ = cv2.findContours(mask_black[wr_y1:wr_y2, wr_x1:wr_x2], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    left_wall_area = max([cv2.contourArea(c) for c in left_wall_contours], default=0)
    right_wall_area = max([cv2.contourArea(c) for c in right_wall_contours], default=0)

    # 2. Find Closest Pillar
    closest_pillar = Pillar()
    num_red, num_green = 0, 0
    screen_bottom_center = (w // 2, h)
    p_x1, p_y1, p_x2, p_y2 = rois['pillar']
    
    # Process Red Pillars
    red_contours, _ = cv2.findContours(mask_red[p_y1:p_y2, p_x1:p_x2], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in red_contours:
        if cv2.contourArea(c) > 400:
            num_red += 1
            M = cv2.moments(c)
            if M["m00"] != 0:
                pX = int(M["m10"] / M["m00"]) + p_x1 # Add ROI offset
                pY = int(M["m01"] / M["m00"]) + p_y1
                dist = math.hypot(pX - screen_bottom_center[0], pY - screen_bottom_center[1])
                if dist < closest_pillar.distance:
                    closest_pillar.distance = dist
                    closest_pillar.area = cv2.contourArea(c)
                    closest_pillar.x, closest_pillar.y = pX, pY
                    closest_pillar.color = 'red'

    # Process Green Pillars
    green_contours, _ = cv2.findContours(mask_green[p_y1:p_y2, p_x1:p_x2], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in green_contours:
        if cv2.contourArea(c) > 400:
            num_green += 1
            M = cv2.moments(c)
            if M["m00"] != 0:
                pX = int(M["m10"] / M["m00"]) + p_x1
                pY = int(M["m01"] / M["m00"]) + p_y1
                dist = math.hypot(pX - screen_bottom_center[0], pY - screen_bottom_center[1])
                if dist < closest_pillar.distance:
                    closest_pillar.distance = dist
                    closest_pillar.area = cv2.contourArea(c)
                    closest_pillar.x, closest_pillar.y = pX, pY
                    closest_pillar.color = 'green'

    # 3. Find Colored Line Areas for Cornering
    l_x1, l_y1, l_x2, l_y2 = rois['line']
    blue_contours, _ = cv2.findContours(mask_blue[l_y1:l_y2, l_x1:l_x2], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    orange_contours, _ = cv2.findContours(mask_orange[l_y1:l_y2, l_x1:l_x2], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blue_line_area = max([cv2.contourArea(c) for c in blue_contours], default=0)
    orange_line_area = max([cv2.contourArea(c) for c in orange_contours], default=0)

    # --- Package and Return All Data ---
    scene_data = {
        "closest_pillar": closest_pillar,
        "num_red": num_red,
        "num_green": num_green,
        "left_wall_area": left_wall_area,
        "right_wall_area": right_wall_area,
        "blue_line_area": blue_line_area,
        "orange_line_area": orange_line_area,
    }

    # --- Draw Debugging UI on the frame ---
    for roi_name, (x1, y1, x2, y2) in rois.items():
        cv2.rectangle(processed_frame, (x1, y1), (x2, y2), (255, 204, 0), 1)
        cv2.putText(processed_frame, roi_name, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 204, 0), 1)
    
    if closest_pillar.color:
        color_bgr = (0, 255, 0) if closest_pillar.color == 'green' else (0, 0, 255)
        cv2.circle(processed_frame, (closest_pillar.x, closest_pillar.y), 15, color_bgr, 3)

    return processed_frame, scene_data