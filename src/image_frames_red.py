# image_frame_test1.py
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
    
    x1 = max(0, x1)
    y1 = max(0, y1)
    x2 = min(frame.shape[1], x2)
    y2 = min(frame.shape[0], y2)

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
    Processes a single camera frame to determine robot steering based on:
    1. Red obstacle detection (primary) -> Steers RIGHT
    2. Black line centering (fallback if no obstacle)
    3. Corner avoidance (fallback if no obstacle or line)
    """
    if frame is None:
        return None, 0, None, "none", 90

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h, w = frame.shape[:2]

    arena_x1 = int(0.2 * w)
    arena_x2 = int(0.8 * w)
    arena_y1 = int(0.2 * h)
    arena_y2 = int(0.8 * h)

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    binary_red_mask = cv2.bitwise_or(mask_red1, mask_red2)

    processed = frame.copy()
    steering_angle = 0
    logic_label = "none"

    outer_start = (int(0.15 * w), int(0.15 * h))
    outer_end = (int(0.85 * w), int(0.85 * h))
    inner_start = (int(0.15 * w), int(0.25 * h))
    inner_end = (int(0.85 * w), int(0.75 * h))
    
    target_x = inner_start[0] 

    contours, _ = cv2.findContours(binary_red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_contours = []

    for cnt in contours:
        mask = np.zeros_like(binary_red_mask)
        cv2.drawContours(mask, [cnt], -1, 255, -1)
        overlap = np.sum(mask[arena_y1:arena_y2, arena_x1:arena_x2]) / 255
        if overlap > 200:
            valid_contours.append(cnt)

    if valid_contours:
        largest = max(valid_contours, key=cv2.contourArea)
        
        # ###############################################################################
        # ## MODIFIED: Use the right-most point of the contour as the reference, not the bottom point.
        # ## This ensures we measure the part of the obstacle that is furthest to the right,
        # ## preventing collisions when the robot turns.
        # ## We find the point with the maximum X-coordinate by using argmax() on index 0.
        reference_point = tuple(largest[largest[:, :, 0].argmax()][0])
        pX, pY = reference_point # pX = x-coordinate, pY = y-coordinate
        # ###############################################################################

        error = pX - target_x
        steering_angle = KP_STEERING * error
        logic_label = "obstacle"

        corner_margin = int(0.15 * w)
        corner_bias = 0
        if pX < corner_margin and pY < corner_margin: corner_bias = 30
        elif pX > w - corner_margin and pY < corner_margin: corner_bias = -30
        elif pX < corner_margin and pY > h - corner_margin: corner_bias = 30
        elif pX > w - corner_margin and pY > h - corner_margin: corner_bias = -30
        steering_angle += corner_bias

        cv2.drawContours(processed, [largest], -1, (0, 0, 255), 2)
        # Draw the circle at the new reference point (the right-most point)
        cv2.circle(processed, (pX, pY), 7, (0, 255, 255), -1) # Yellow circle for visibility
        cv2.line(processed, (pX, pY), (target_x, pY), (255, 0, 0), 2)

    if logic_label == "none":
        correction = analyze_black_between_lines(frame, inner_start, inner_end)
        if correction is not None:
            steering_angle = correction
            logic_label = "line_centering"

    if logic_label == "none":
        center_x, center_y = w // 2, h // 2
        in_top_left = center_x < 0.3 * w and center_y < 0.3 * h
        in_top_right = center_x > 0.7 * w and center_y < 0.3 * h
        in_bottom_left = center_x < 0.3 * w and center_y > 0.7 * h
        in_bottom_right = center_x > 0.7 * w and center_y > 0.7 * h
        
        virtual_target_x = None
        if in_top_left or in_bottom_left:
            virtual_target_x = int(0.7 * w)
            logic_label = "corner_left_avoid"
        elif in_top_right or in_bottom_right:
            virtual_target_x = int(0.3 * w)
            logic_label = "corner_right_avoid"
        
        if virtual_target_x is not None:
            error = virtual_target_x - center_x
            steering_angle = KP_STEERING * error

    cv2.rectangle(processed, outer_start, outer_end, (0, 255, 0), 2)
    cv2.rectangle(processed, inner_start, inner_end, (0, 255, 0), 2)
    cv2.line(processed, (w // 2, inner_start[1]), (w // 2, inner_end[1]), (0, 255, 0), 2)
    cv2.line(processed, (target_x, inner_start[1]), (target_x, inner_end[1]), (255, 255, 0), 2)

    return processed, steering_angle, binary_red_mask, logic_label, 0