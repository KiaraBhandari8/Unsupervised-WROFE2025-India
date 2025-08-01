
import cv2
import numpy as np

KP_STEERING = 0.5
KP_LINE_CENTERING = 0.4

def analyze_black_between_lines(frame, inner_start, inner_end):
    """
    Analyzes the amount of black pixels between two vertical lines (inner_start, inner_end)
    to determine a correction for line centering.
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    x1, y1 = inner_start
    x2, y2 = inner_end
    
    # Ensure ROI coordinates are within frame bounds
    x1 = max(0, x1)
    y1 = max(0, y1)
    x2 = min(frame.shape[1], x2)
    y2 = min(frame.shape[0], y2)

    if x1 >= x2 or y1 >= y2:
        return None # Invalid ROI

    roi = gray[y1:y2, x1:x2]
    
    # Threshold to find black areas (pixels darker than 60)
    _, black_mask = cv2.threshold(roi, 60, 255, cv2.THRESH_BINARY_INV)

    h, w = black_mask.shape
    
    # Divide the ROI into left and right halves
    left = black_mask[:, :w // 2]
    right = black_mask[:, w // 2:]

    # Calculate the sum of black pixels in each half
    black_left = np.sum(left) / 255 # Divide by 255 to get count of black pixels
    black_right = np.sum(right) / 255
    total_black = black_left + black_right

    if total_black == 0:
        return None # No black detected, no correction needed

    # Calculate balance to determine if more black is on left or right
    balance = (black_right - black_left) / total_black
    
    # Calculate correction. Positive correction means steer right, negative means steer left.
    # Multiplied by 100 for a larger range of steering values.
    correction = KP_LINE_CENTERING * balance * 100
    return correction

def process_frame_for_steering(frame):
    """
    Processes a single camera frame to determine robot steering based on:
    1. Green obstacle detection (primary)
    2. Black line centering (fallback if no obstacle)
    3. Corner avoidance (fallback if no obstacle or line)

    Args:
        frame (numpy.ndarray): The input BGR image frame from the camera.

    Returns:
        tuple: (processed_frame, steering_angle, binary_green_mask, logic_label, unused_value)
            - processed_frame: The frame with UI overlays for debugging.
            - steering_angle: The calculated steering angle (positive for right, negative for left).
            - binary_green_mask: The binary mask of detected green objects.
            - logic_label: A string indicating the active behavior ("obstacle", "line_centering",
                           "corner_left_avoid", "corner_right_avoid", "none").
            - unused_value: Always 0, for compatibility with original function signature.
    """
    if frame is None:
        # Return default values if frame is invalid
        return None, 0, None, "none", 90

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = frame.shape[:2]

    # Define arena region for valid obstacle detection (central 60% of width and height)
    arena_x1 = int(0.2 * w)
    arena_x2 = int(0.8 * w)
    arena_y1 = int(0.2 * h)
    arena_y2 = int(0.8 * h)

    # Define color range for green obstacles in HSV
    lower_green = np.array([35, 100, 50])
    upper_green = np.array([85, 255, 255])
    binary_green = cv2.inRange(hsv, lower_green, upper_green)

    processed = frame.copy() # Create a copy to draw overlays on
    steering_angle = 0
    logic_label = "none" # Default state

    # UI overlays for debugging: Define regions of interest
    outer_start = (int(0.15 * w), int(0.15 * h))
    outer_end = (int(0.85 * w), int(0.85 * h))
    inner_start = (int(0.15 * w), int(0.25 * h))
    inner_end = (int(0.85 * w), int(0.75 * h))
    target_x = inner_end[0] # Target X-coordinate for obstacle centering

    # Find contours of green objects
    contours, _ = cv2.findContours(binary_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_contours = []

    # Filter contours to only include those within the defined arena region
    for cnt in contours:
        mask = np.zeros_like(binary_green)
        cv2.drawContours(mask, [cnt], -1, 255, -1)
        # Check overlap with the arena region
        overlap = np.sum(mask[arena_y1:arena_y2, arena_x1:arena_x2]) / 255
        if overlap > 200: # If a significant part of the contour is in the arena
            valid_contours.append(cnt)

    if valid_contours:
        # If green obstacles are detected, prioritize obstacle avoidance
        largest = max(valid_contours, key=cv2.contourArea) # Get the largest green contour
        
        # Find the bottom-most point of the largest contour
        bottom_point = tuple(largest[largest[:, :, 1].argmax()][0])
        pX, pY = bottom_point # pX = x-coordinate, pY = y-coordinate

        # Calculate error: difference between target_x and the obstacle's x-position
        # Positive error means obstacle is to the left of target, needs right turn
        error = target_x - pX
        steering_angle = KP_STEERING * error
        logic_label = "obstacle"

        # Add a bias for corner avoidance if the obstacle is near a corner
        corner_margin = int(0.15 * w) # 15% of width as corner margin
        corner_bias = 0
        
        # Check if the obstacle's bottom point is in any of the four corners
        # This logic seems to apply a fixed bias regardless of which corner,
        # which might need fine-tuning for specific robot behaviors.
        if pX < corner_margin and pY < corner_margin: # Top-left
            corner_bias = 30
        elif pX > w - corner_margin and pY < corner_margin: # Top-right
            corner_bias = -30
        elif pX < corner_margin and pY > h - corner_margin: # Bottom-left
            corner_bias = 30
        elif pX > w - corner_margin and pY > h - corner_margin: # Bottom-right
            corner_bias = -30
        steering_angle += corner_bias

        # Draw obstacle visualization on the processed frame
        cv2.drawContours(processed, [largest], -1, (0, 0, 255), 2) # Red contour
        cv2.circle(processed, (pX, pY), 7, (0, 0, 255), -1) # Red circle at bottom point
        cv2.line(processed, (pX, pY), (target_x, pY), (255, 0, 0), 2) # Blue line from obstacle to target X

    # --- Line Following ---
    # If no obstacle is detected, try line centering
    if logic_label == "none":
        correction = analyze_black_between_lines(frame, inner_start, inner_end)
        if correction is not None:
            steering_angle = correction
            logic_label = "line_centering"

    # --- Corner Avoidance ---
    # If no obstacle or line is detected, try corner avoidance
    if logic_label == "none":
        center_x = w // 2
        center_y = h // 2

        # Check if the center of the frame (where the robot is likely looking)
        # is within a corner region.
        in_top_left = center_x < 0.3 * w and center_y < 0.3 * h
        in_top_right = center_x > 0.7 * w and center_y < 0.3 * h
        in_bottom_left = center_x < 0.3 * w and center_y > 0.7 * h
        in_bottom_right = center_x > 0.7 * w and center_y > 0.7 * h

        virtual_target_x = None

        if in_top_left or in_bottom_left:
            # If in left corners, set a virtual target to the right to turn right
            virtual_target_x = int(0.7 * w)
            logic_label = "corner_left_avoid"
        elif in_top_right or in_bottom_right:
            # If in right corners, set a virtual target to the left to turn left
            virtual_target_x = int(0.3 * w)
            logic_label = "corner_right_avoid"
        else:
            virtual_target_x = None

        if virtual_target_x is not None:
            # Calculate steering based on error from the virtual target
            error = virtual_target_x - center_x
            steering_angle = KP_STEERING * error

    # UI overlays (drawn regardless of active logic for visual debugging)
    cv2.rectangle(processed, outer_start, outer_end, (0, 255, 0), 2) # Green outer rectangle
    cv2.rectangle(processed, inner_start, inner_end, (0, 255, 0), 2) # Green inner rectangle
    cv2.line(processed, (int(0.5 * w), inner_start[1]), (int(0.5 * w), inner_end[1]), (0, 255, 0), 2) # Green center line
    cv2.line(processed, (inner_end[0], inner_start[1]), inner_end, (255, 255, 0), 2) # Yellow line to target_x (for obstacle)

    return processed, steering_angle, binary_green, logic_label, 0
