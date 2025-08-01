import cv2
import numpy as np

def get_steering_command(frame):
    """
    Returns (command, steering_angle, visualization_frame)
    command: "FORWARD", "LEFT", "RIGHT", or "STOP"
    steering_angle: float, positive = right, negative = left, 0 = straight
    """
    h, w, _ = frame.shape

    # Convert to HSV and threshold for path (example: white path)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([0, 0, 180])
    upper = np.array([180, 40, 255])
    mask = cv2.inRange(hsv, lower, upper)

    # Morphology to clean up
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find largest contour (assume this is the path)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return "STOP", None, frame

    largest = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return "STOP", None, frame

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    image_center = w // 2

    # Calculate steering angle (simple proportional controller)
    offset = cx - image_center
    steering_angle = (offset / (w / 2)) * 45  # Max 45 degrees

    # Decide command
    tolerance = w // 20
    if abs(offset) < tolerance:
        command = "FORWARD"
    elif offset < 0:
        command = "LEFT"
    else:
        command = "RIGHT"

    # Visualization
    vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    cv2.drawContours(vis, [largest], -1, (0, 255, 0), 2)
    cv2.circle(vis, (cx, cy), 10, (0, 0, 255), -1)
    cv2.line(vis, (image_center, 0), (image_center, h), (255, 0, 0), 2)
    cv2.putText(vis, f"{command} ({steering_angle:.1f})", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)

    return command, steering_angle, vis
