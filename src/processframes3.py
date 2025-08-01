import cv2
import numpy as np

def calculate_steering_angle_centroid(cx_full, image_center_x, w, tolerance=20, max_angle=50):
    centroid_offset = cx_full - image_center_x
    if abs(centroid_offset) > tolerance:
        steering_angle = centroid_offset / (w / 2) * max_angle
    else:
        steering_angle = 0
    return steering_angle

def calculate_steering_angle_area(filtered_path_mask, max_angle=50):
    h, w = filtered_path_mask.shape
    left_area = np.sum(filtered_path_mask[:, :w//2] > 0)
    right_area = np.sum(filtered_path_mask[:, w//2:] > 0)
    total_area = left_area + right_area
    if total_area == 0:
        return 0
    area_diff_ratio = (right_area - left_area) / total_area
    steering_angle = area_diff_ratio * max_angle
    return steering_angle

def get_robot_direction_and_angle(frame):
    if frame is None:
        print("Error: Input frame is None.")
        return "STOP", None, None, None

    h, w, _ = frame.shape

    # --- Black Masking (arena boundaries) ---
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 130])
    mask_black = cv2.inRange(hsv, lower_black, upper_black)

    # --- Erosion to clean up mask ---
    kernel = np.ones((5, 5), np.uint8)
    mask_eroded = cv2.erode(mask_black, kernel, iterations=5)

    # --- Find contours on eroded mask (black boundaries) ---
    contours_black_boundaries, _ = cv2.findContours(mask_eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours_black_boundaries:
        print("No black boundaries found. Stopping.")
        return "STOP", None, frame.copy(), frame.copy()

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
        return "STOP", None, frame.copy(), frame.copy()

    # --- Obstacle Detection (Green/Red) ---
    green_lower = np.array([40, 70, 70])
    green_upper = np.array([80, 255, 255])
    mask_green = cv2.inRange(hsv, green_lower, green_upper)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    red_lower1 = np.array([0, 70, 70])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([170, 70, 70])
    red_upper2 = np.array([180, 255, 255])
    mask_red1 = cv2.inRange(hsv, red_lower1, red_upper1)
    mask_red2 = cv2.inRange(hsv, red_lower2, red_upper2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the largest green and red obstacles (if any), closest to image center
    green_bbox = None
    red_bbox = None
    green_center_dist = float('inf')
    red_center_dist = float('inf')
    image_center_x = w // 2

    if contours_green:
        largest_green = max(contours_green, key=cv2.contourArea)
        if cv2.contourArea(largest_green) > 100:  # Lowered threshold for "slight" detection
            xg, yg, wg, hg = cv2.boundingRect(largest_green)
            green_bbox = (xg, yg, wg, hg)
            green_center_dist = abs((xg + wg // 2) - image_center_x)
    if contours_red:
        largest_red = max(contours_red, key=cv2.contourArea)
        if cv2.contourArea(largest_red) > 100:
            xr, yr, wr, hr = cv2.boundingRect(largest_red)
            red_bbox = (xr, yr, wr, hr)
            red_center_dist = abs((xr + wr // 2) - image_center_x)

    # Decide which obstacle is closest to the center
    obstacle_color = None
    obstacle_bbox = None
    if green_bbox and (not red_bbox or green_center_dist < red_center_dist):
        obstacle_color = "green"
        obstacle_bbox = green_bbox
    elif red_bbox:
        obstacle_color = "red"
        obstacle_bbox = red_bbox

    # --- Mask out from obstacle to non-driveable side (including obstacle) ---
    if obstacle_color and obstacle_bbox:
        x, y, w_box, h_box = obstacle_bbox
        mask_mod = np.ones_like(filtered_path_mask, dtype=np.uint8) * 255

        if obstacle_color == "green":
            # Mask from the right edge up to and including the green obstacle
            mask_mod[:, x:] = 0
        elif obstacle_color == "red":
            # Mask from the left edge up to and including the red obstacle
            mask_mod[:, :x + w_box] = 0

        filtered_path_mask = cv2.bitwise_and(filtered_path_mask, mask_mod)

    # --- Centroid Calculation ---
    M = cv2.moments(filtered_path_mask)

    command = "STOP"
    steering_angle = None
    cx_full = None
    cy_full = None

    if M["m00"] != 0:
        cx_full = int(M["m10"] / M["m00"])
        cy_full = int(M["m01"] / M["m00"])

        tolerance = 20
        max_angle = 60

        steering_angle_centroid = calculate_steering_angle_centroid(cx_full, image_center_x, w, tolerance, max_angle)
        steering_angle_area = calculate_steering_angle_area(filtered_path_mask, max_angle)

        steering_angle = 0.5 * steering_angle_centroid + 0.5 * steering_angle_area

        if np.sum(filtered_path_mask) < (w * h * 0.005):
            print("Filtered path mask is mostly black (path likely lost). Stopping.")
            command = "STOP"
            steering_angle = None
            cx_full = None
            cy_full = None
        else:
            if steering_angle < -tolerance:
                command = "LEFT"
            elif steering_angle > tolerance:
                command = "RIGHT"
            else:
                command = "FORWARD"
    else:
        print("No significant path found in filtered mask. Stopping.")
        command = "STOP"
        steering_angle = None

    # --- Visualization Frame ---
    visualized_frame = cv2.cvtColor(filtered_path_mask, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(visualized_frame, contours_black_boundaries, -1, (0, 255, 0), 3)
    if relevant_path_contour is not None:
        cv2.drawContours(visualized_frame, [relevant_path_contour], -1, (255, 0, 255), 2)

    if cx_full is not None and cy_full is not None:
        cv2.circle(visualized_frame, (cx_full, cy_full), 10, (0, 0, 255), -1)
        cv2.line(visualized_frame, (w // 2, 0), (w // 2, h), (255, 0, 0), 2)
        if command in ["FORWARD", "LEFT", "RIGHT"] and steering_angle is not None:
            arrow_start_x = cx_full
            arrow_start_y = cy_full
            arrow_length = 50
            plot_angle_degrees = -90 + steering_angle
            plot_angle_rad = np.deg2rad(plot_angle_degrees)
            arrow_end_x = int(arrow_start_x + arrow_length * np.cos(plot_angle_rad))
            arrow_end_y = int(arrow_start_y + arrow_length * np.sin(plot_angle_rad))
            cv2.arrowedLine(visualized_frame, (arrow_start_x, arrow_start_y),
                            (arrow_end_x, arrow_end_y), (255, 255, 0), 4, tipLength=0.5)

    if obstacle_bbox:
        x, y, w_box, h_box = obstacle_bbox
        color = (0,255,0) if obstacle_color == "green" else (0,0,255)
        cv2.rectangle(visualized_frame, (x, y), (x+w_box, y+h_box), color, 3)
        cv2.putText(visualized_frame, obstacle_color.upper(), (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

    if steering_angle is not None:
        steering_angle = int(steering_angle)
    driveable_mask_img = cv2.cvtColor(filtered_path_mask, cv2.COLOR_GRAY2BGR)
    return command, steering_angle, visualized_frame, driveable_mask_img

