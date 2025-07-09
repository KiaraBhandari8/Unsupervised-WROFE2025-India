import cv2
import numpy as np

def calculate_steering_angle_centroid(cx_full, image_center_x, w, tolerance=20, max_angle=50):
    """
    Calculate steering angle based on centroid offset.
    """
    centroid_offset = cx_full - image_center_x
    if abs(centroid_offset) > tolerance:
        steering_angle = centroid_offset / (w / 2) * max_angle
    else:
        steering_angle = 0
    return steering_angle

def calculate_steering_angle_area(filtered_path_mask, max_angle=50):
    """
    Calculate steering angle based on white area (path) on left and right halves.
    If more white area is on the left, returns negative angle (turn left).
    If more white area is on the right, returns positive angle (turn right).
    """
    h, w = filtered_path_mask.shape
    left_area = np.sum(filtered_path_mask[:, :w//2] > 0)
    right_area = np.sum(filtered_path_mask[:, w//2:] > 0)
    total_area = left_area + right_area
    if total_area == 0:
        return 0  # No path detected

    # The difference ratio determines the angle, scaled to max_angle
    area_diff_ratio = (right_area - left_area) / total_area
    steering_angle = area_diff_ratio * max_angle
    return steering_angle

def get_robot_direction_and_angle(frame):
    """
    Analyzes an input image frame to determine robot movement direction and steering angle.
    Returns the direction, steering angle, and a visualized frame with annotations.

    Args:
        frame (numpy.ndarray): The input image frame (BGR format).

    Returns:
        tuple: A tuple containing:
            - str: Movement command ("FORWARD", "LEFT", "RIGHT", "STOP").
            - float or None: Recommended steering angle in degrees (positive for right, negative for left, 0 for forward).
                             None if command is "STOP".
            - numpy.ndarray: The input frame with centroid (red circle) and robot heading (yellow arrow) drawn.
                             Returns None if the input frame is invalid.
    """
    if frame is None:
        print("Error: Input frame is None.")
        return "STOP", None, None

    h, w, _ = frame.shape

    # --- Black Masking ---
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
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
        return "STOP", None, frame.copy() # Return original frame if nothing found

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
        return "STOP", None, frame.copy()

    # --- Centroid Calculation ---
    M = cv2.moments(filtered_path_mask)

    command = "STOP"
    steering_angle = None
    cx_full = None
    cy_full = None

    if M["m00"] != 0:
        cx_full = int(M["m10"] / M["m00"])
        cy_full = int(M["m01"] / M["m00"])

        image_center_x = w // 2
        tolerance = 20
        max_angle = 60

        # Calculate steering angles
        steering_angle_centroid = calculate_steering_angle_centroid(cx_full, image_center_x, w, tolerance, max_angle)
        steering_angle_area = calculate_steering_angle_area(filtered_path_mask, max_angle)

        print(f"Reocommended Steering Angle from Centoriod : {steering_angle_centroid}")
        print(f"Reocommended Steering Angle from Area : {steering_angle_area}")

        # Combine both (simple average, you can adjust weighting as needed)
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
    # visualized_frame = frame.copy()
    visualized_frame = filtered_path_mask

    # Draw the detected black boundaries (green)
    cv2.drawContours(visualized_frame, contours_black_boundaries, -1, (0, 255, 0), 3)

    # Draw the path contour that was used for centroid calculation (magenta)
    if relevant_path_contour is not None:
        cv2.drawContours(visualized_frame, [relevant_path_contour], -1, (255, 0, 255), 2)

    # Draw the calculated centroid and reference line
    if cx_full is not None and cy_full is not None:
        cv2.circle(visualized_frame, (cx_full, cy_full), 50, (0, 0, 255), -1) # Red circle for centroid
        cv2.line(visualized_frame, (w // 2, 0), (w // 2, h), (255, 0, 0), 5) # Blue center line

        # Add directional marker (yellow arrow) if moving FORWARD, LEFT, or RIGHT
        if command in ["FORWARD", "LEFT", "RIGHT"] and steering_angle is not None:
            arrow_start_x = cx_full
            arrow_start_y = cy_full
            arrow_length = 50

            # Angle for drawing the arrow:
            # -90 degrees for straight up (forward) in OpenCV's Y-down coordinate system.
            # Add positive 'steering_angle' for right turns (makes arrow lean right).
            # Subtract negative 'steering_angle' for left turns (makes arrow lean left).
            # Since steering_angle is positive for right and negative for left, we subtract:
            plot_angle_degrees = -90 + steering_angle
            plot_angle_rad = np.deg2rad(plot_angle_degrees)

            arrow_end_x = int(arrow_start_x + arrow_length * np.cos(plot_angle_rad))
            arrow_end_y = int(arrow_start_y + arrow_length * np.sin(plot_angle_rad))

            cv2.arrowedLine(visualized_frame, (arrow_start_x, arrow_start_y),
                            (arrow_end_x, arrow_end_y), (255, 255, 0), 10, tipLength=0.5)
    steering_angle = int(steering_angle)
    return command, steering_angle, visualized_frame



if __name__ == "__main__":
    import sys

    # Example usage: python process_frames.py [image_path]
    if len(sys.argv) > 1:
        image_path = sys.argv[1]
    else:
        # Default test image path
        image_path = "test.jpg"

    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Failed to load image: {image_path}")
        sys.exit(1)

    command, steering_angle, visualized_frame = get_robot_direction_and_angle(frame)
    print(f"Command: {command}")
    print(f"Steering Angle: {steering_angle}")

    if visualized_frame is not None:
        output_path = "result_annotated.jpg"
        cv2.imwrite(output_path, visualized_frame)
        print(f"Result image saved to {output_path}")
    else:
        print("No visualization available.")
