# image_frame_test2.py
import cv2
import numpy as np

# --- PID GAINS ---
KP_STEERING = 0.5
KP_LINE_CENTERING = 0.4

# --- SHAPE FILTERING CONSTANTS ---
# The object's height must be at least 1.2 times its width to be considered a pillar.
# Increase this value to be more selective (e.g., 1.5 or 2.0).
# Decrease it to be less selective.
PILLAR_ASPECT_RATIO_THRESHOLD = 1.2

def analyze_black_between_lines(frame, inner_start, inner_end):
    """
    Analyzes the amount of black pixels between two vertical lines (inner_start, inner_end)
    to determine a correction for line centering.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    x1, y1 = inner_start
    x2, y2 = inner_end
    
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(frame.shape[1], x2), min(frame.shape[0], y2)

    if x1 >= x2 or y1 >= y2:
        return None

    roi = gray[y1:y2, x1:x2]
    _, black_mask = cv2.threshold(roi, 60, 255, cv2.THRESH_BINARY_INV)
    h, w = black_mask.shape
    
    left = black_mask[:, :w // 2]
    right = black_mask[:, w // 2:]

    black_left = np.sum(left) / 255
    black_right = np.sum(right) / 255
    total_black = black_left + black_right

    if total_black == 0:
        return None

    balance = (black_right - black_left) / total_black
    correction = KP_LINE_CENTERING * balance * 100
    return correction

def process_frame_for_steering(frame):
    """
    Processes a single camera frame to determine robot steering.
    The logic for red and green obstacles is now based on the center-bottom of the pillar's bounding box.
    """
    if frame is None:
        return None, 0, None, "none", 0

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = frame.shape[:2]
    processed = frame.copy()
    steering_angle, logic_label = 0, "none"
    center_x = w // 2

    # --- COLOR DEFINITIONS ---
    lower_green = np.array([35, 100, 50])
    upper_green = np.array([85, 255, 255])
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # --- MASKS ---
    binary_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    binary_red = cv2.bitwise_or(mask_red1, mask_red2)
    output_mask = binary_green

    # --- UI & LOGIC REGIONS ---
    outer_start = (int(0.15 * w), int(0.15 * h))
    outer_end = (int(0.85 * w), int(0.85 * h))
    inner_start = (int(0.15 * w), int(0.25 * h))
    inner_end = (int(0.85 * w), int(0.75 * h))
    
    green_target_x = inner_end[0]
    red_target_x = int(0.05 * w)   

    # --- OBSTACLE DETECTION LOGIC ---
    obstacle_contour, obstacle_bottom_point, obstacle_target_x = None, None, None

    # Behavior 1: Red Obstacle (Highest Priority)
    red_contours, _ = cv2.findContours(binary_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if red_contours:
        largest_red = max(red_contours, key=cv2.contourArea)
        if cv2.contourArea(largest_red) > 500:
            # Get the bounding box for aspect ratio check AND for stable positioning
            x_b, y_b, w_b, h_b = cv2.boundingRect(largest_red)
            aspect_ratio = float(h_b) / w_b if w_b > 0 else 0

            # Only proceed if the object is taller than it is wide (a pillar)
            if aspect_ratio > PILLAR_ASPECT_RATIO_THRESHOLD:
                
                # <<< CHANGED: Calculate reference point from the bottom-middle of the bounding box
                pX_red = x_b + w_b // 2  # Horizontal center of the pillar
                pY_red = y_b + h_b       # Bottom of the pillar
                
                if pY_red >= inner_start[1]:
                    logic_label = "red_obstacle"
                    output_mask = binary_red
                    
                    # Error is calculated from the pillar's center (pX_red)
                    error = red_target_x - pX_red
                    steering_angle = KP_STEERING * error
                    
                    obstacle_contour = largest_red
                    obstacle_bottom_point = (pX_red, pY_red) # Use the new stable point for visualization
                    obstacle_target_x = red_target_x

    # Behavior 2: Green Obstacle
    if logic_label == "none":
        green_contours, _ = cv2.findContours(binary_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if green_contours:
            largest_green = max(green_contours, key=cv2.contourArea)
            if cv2.contourArea(largest_green) > 500:
                logic_label = "obstacle"
                
                # <<< NEW: Get bounding box for the green contour to find its stable center
                x_b, y_b, w_b, h_b = cv2.boundingRect(largest_green)
                
                # <<< CHANGED: Calculate reference point from the bottom-middle of the bounding box
                pX_green = x_b + w_b // 2 # Horizontal center of the pillar
                pY_green = y_b + h_b      # Bottom of the pillar

                # Error is calculated from the pillar's center (pX_green)
                error = green_target_x - pX_green
                steering_angle = KP_STEERING * error
                
                obstacle_contour = largest_green
                obstacle_bottom_point = (pX_green, pY_green) # Use the new stable point for visualization
                obstacle_target_x = green_target_x

    # Behavior 3 & 4: Fallback Logic
    if logic_label == "none":
        correction = analyze_black_between_lines(frame, inner_start, inner_end)
        if correction is not None:
            steering_angle = correction
            logic_label = "line_centering"
        else:
             gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
             if np.mean(gray_frame) < 50:
                  steering_angle = -45
                  logic_label = "corner_avoid"

    # --- FINAL UI OVERLAYS ---
    cv2.rectangle(processed, outer_start, outer_end, (0, 255, 0), 2)
    cv2.rectangle(processed, inner_start, inner_end, (0, 255, 0), 2)
    cv2.line(processed, (center_x, inner_start[1]), (center_x, inner_end[1]), (0, 255, 0), 2)
    cv2.line(processed, (green_target_x, inner_start[1]), (green_target_x, inner_end[1]), (255, 255, 0), 2)

    if obstacle_contour is not None:
        pX, pY = obstacle_bottom_point
        cv2.drawContours(processed, [obstacle_contour], -1, (0, 0, 255), 2)
        # The circle now marks the stable bottom-center point
        cv2.circle(processed, (pX, pY), 7, (0, 0, 255), -1)
        # The blue error line now correctly originates from this stable point
        cv2.line(processed, (pX, pY), (obstacle_target_x, pY), (255, 0, 0), 2)

    return processed, steering_angle, output_mask, logic_label, 0