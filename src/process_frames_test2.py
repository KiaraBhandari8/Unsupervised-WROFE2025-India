import cv2
import numpy as np

def calculate_steering_angle_centroid(cx, image_center_x, w, tolerance=20, max_angle=50):
    offset = cx - image_center_x
    if abs(offset) > tolerance:
        angle = offset / (w / 2) * max_angle
    else:
        angle = 0 
    return angle

def calculate_steering_angle_area(mask, max_angle=50):
    h, w = mask.shape
    left = np.sum(mask[:, :w // 2] > 0)
    right = np.sum(mask[:, w // 2:] > 0)
    total = left + right
    if total == 0:
        return 0
    ratio = (right - left) / total
    print(f"Total : {total}, Left : {left}, Right : {right}")
    return ratio * max_angle

def get_robot_direction_and_angle(frame, facing_original_direction=False):
    if frame is None:
        print("Frame is None.")
        return "STOP", None, None, 0, 0

    h, w, _ = frame.shape
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 255, 130])
    mask_black = cv2.inRange(hsv, lower_black, upper_black)

    kernel = np.ones((5, 5), np.uint8)
    mask_eroded = cv2.erode(mask_black, kernel, iterations=5)

    contours, _ = cv2.findContours(mask_eroded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return "STOP", None, frame, 0, 0

    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]
    mask_boundaries = np.zeros_like(mask_eroded)
    cv2.drawContours(mask_boundaries, contours, -1, 255, -1)
    path_mask_initial = cv2.bitwise_not(mask_boundaries)

    bottom_y = int(h * 0.75) if facing_original_direction else int(h * 0.25)
    components, _ = cv2.findContours(path_mask_initial, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    filtered_mask = np.zeros_like(path_mask_initial)
    selected = None
    max_area = 0

    for c in components:
        _, y, _, hc = cv2.boundingRect(c)
        if facing_original_direction:
            if (y + hc) >= bottom_y:
                area = cv2.contourArea(c)
                if area > max_area:
                    max_area = area
                    selected = c
        else:
            if y <= bottom_y:
                area = cv2.contourArea(c)
                if area > max_area:
                    max_area = area
                    selected = c

    if selected is not None:
        cv2.drawContours(filtered_mask, [selected], -1, 255, -1)
    else:
        return "STOP", None, frame, 0, 0

    M = cv2.moments(filtered_mask)
    command = "STOP"
    steering_angle = None
    angle_centroid = 0
    angle_area = 0

    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        center_x = w // 2
        angle_centroid = calculate_steering_angle_centroid(cx, center_x, w)
        print(f"Angle Centroid : {angle_centroid}")
        angle_area = calculate_steering_angle_area(filtered_mask)
        print(f"Angle Area : {angle_area}")
        steering_angle = int(0.5 * angle_centroid + 1* angle_area)

        if np.sum(filtered_mask) < (w * h * 0.005):
            command = "STOP"
            steering_angle = None
        else:
            if steering_angle < -20:
                command = "LEFT"
            elif steering_angle > 20:
                command = "RIGHT"
            else:
                command = "FORWARD"
    else:
        command = "STOP"
        steering_angle = None

    return command, steering_angle, filtered_mask