import cv2
import numpy as np
from picamera2 import Picamera2
import time
import libcamera



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
    print(f"Left Area : {left_area}, Right Area : {right_area}")
    total_area = left_area + right_area
    if total_area == 0:
        return 0  # No path detected

    # The difference ratio determines the angle, scaled to max_angle
    area_diff_ratio = (right_area - left_area) / total_area
    print(f"Area Ratio : {area_diff_ratio}")
    steering_angle = area_diff_ratio * max_angle
    
    fixed_steering_angle = None
    if right_area < 10:
        fixed_steering_angle = -25
    elif left_area < 10:
        fixed_steering_angle = 25
    return steering_angle, fixed_steering_angle


def filter_green_objects(hsv_frame):
    lower_green = np.array([30, 40, 40])
    upper_green = np.array([85, 255, 255])
    green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
    kernel_green = np.ones((5, 5), np.uint8)
    green_mask_eroded = cv2.erode(green_mask, kernel_green, iterations=3)
    green_mask_dilated = cv2.dilate(green_mask_eroded, kernel_green, iterations=1)
    green_signals, _ = cv2.findContours(green_mask_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f"Count of green objects : {len(green_signals)}")
    return len(green_signals), green_signals, green_mask_dilated


def filter_blue_objects(hsv_frame):
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([130, 255, 255])
    blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
    kernel = np.ones((5, 5), np.uint8)
    blue_mask_eroded = cv2.erode(blue_mask, kernel, iterations=2)
    blue_mask_dilated = cv2.dilate(blue_mask_eroded, kernel, iterations=2)
    blue_contours, _ = cv2.findContours(blue_mask_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)    
    print(f"Count of blue objects: {len(blue_contours)}")    
    return len(blue_contours), blue_contours, blue_mask_dilated


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

def get_robot_direction_and_angle(frame):

    if frame is None:
        print("Error: Input frame is None.")
        return "STOP", None, None

    h, w, _ = frame.shape
    print(f"Height : {h}, Width : {w}")

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

    current_drivable_path = filtered_path_mask
    green_count, green_signals, green_mask_dilated = filter_green_objects(hsv)
    if green_count > 0:
        largest_green_contour = max(green_signals, key=cv2.contourArea)
        print(f"Green Signals Detected : {green_count}")
        avg_left_x, avg_top_y, bbox = find_border_coordinates(largest_green_contour)
        y_coords, x_coords = np.indices(current_drivable_path.shape)
        region_to_zero_mask = (x_coords > avg_left_x) 
        current_drivable_path[region_to_zero_mask] = 0

        bottom_25_percent_y = int(h * 0.75)
        contours_path_components, _ = cv2.findContours(current_drivable_path, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        zero_mask = np.zeros_like(current_drivable_path)
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
            cv2.drawContours(zero_mask, [relevant_path_contour], -1, 255, -1)
        else:
            print("No significant path component found in the bottom 25% of the frame. Stopping.")
            return "STOP", None, frame.copy()

    if green_count > 0:
        current_drivable_path = zero_mask
    else:
        current_drivable_path = filtered_path_mask
    # --- Centroid Calculation ---
    M = cv2.moments(current_drivable_path)

    command = "STOP"
    steering_angle = None
    cx_full = None
    cy_full = None

    if M["m00"] != 0:
        cx_full = int(M["m10"] / M["m00"])
        cy_full = int(M["m01"] / M["m00"])

        image_center_x = w // 2
        tolerance = 20
        max_angle = 45

        # Calculate steering angles
        # steering_angle_centroid = calculate_steering_angle_centroid(cx_full, image_center_x, w, tolerance, max_angle)
        steering_angle_area, fixed_steering_angle = calculate_steering_angle_area(current_drivable_path, max_angle)
        
        if fixed_steering_angle is not None:
            steering_angle_area = fixed_steering_angle
        # print(f"Reocommended Steering Angle from Centoriod : {steering_angle_centroid}")
        print(f"Reocommended Steering Angle from Area : {steering_angle_area}")

        # Combine both (simple average, you can adjust weighting as needed)
        # steering_angle = 0.5 * steering_angle_centroid + 0.5 * steering_angle_area
        steering_angle = steering_angle_area

        if np.sum(current_drivable_path) < (w * h * 0.005):
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
    visualized_frame = current_drivable_path.copy()

    # Draw the detected black boundaries (green)
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

    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(main={"size": (768, 432)},
                                                            transform=libcamera.Transform(vflip=True, hflip=True))
    picam2.configure(camera_config)
    picam2.start()
    time.sleep(2)  # Camera warm-up

    frame = picam2.capture_array()
    cv2.imwrite("input_image.jpg", frame)

    command, steering_angle, visualized_frame = get_robot_direction_and_angle(frame)
    print(f"Command: {command}")
    print(f"Steering Angle: {steering_angle}")

    if visualized_frame is not None:
        output_path = "result_annotated.jpg"
        cv2.imwrite(output_path, visualized_frame)
        print(f"Result image saved to {output_path}")
    else:
        print("No visualization available.")
