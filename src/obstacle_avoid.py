import cv2
import numpy as np

def detect_pillar_color_and_position(frame):
  
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


    lower_green = np.array([40, 70, 70])
    upper_green = np.array([85, 255, 255])

    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # Create masks
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # --- Image Cleaning (Morphological Operations) ---
    # Kernel for morphological operations (e.g., 5x5 square)
    kernel = np.ones((5,5),np.uint8)

    # Apply morphological opening to remove small noise (erosion followed by dilation)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

    # Apply morphological closing to fill small holes in detected objects (dilation followed by erosion)
    mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

    detected_color = None
    center_x = None

    # --- Contour Detection and Filtering ---
    # Minimum area for a contour to be considered a pillar
    # Adjust this based on how large a pillar appears in pixels at detection distance.
    MIN_PILLAR_AREA = 200 # Example value, needs calibration for your setup

    # Process Green
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours_green:
        largest_contour_green = None
        max_area_green = 0
        for cnt in contours_green:
            area = cv2.contourArea(cnt)
            if area > MIN_PILLAR_AREA:
                if area > max_area_green:
                    max_area_green = area
                    largest_contour_green = cnt
        
        if largest_contour_green is not None:
            M = cv2.moments(largest_contour_green)
            if M["m00"] != 0: # Avoid division by zero
                center_x = int(M["m10"] / M["m00"])
                detected_color = 'green'
                
                # Draw bounding box and text for debugging in main.py (optional, handled in main.py)
                # x, y, w, h = cv2.boundingRect(largest_contour_green)
                # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # cv2.putText(frame, "Green", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                return detected_color, center_x

    # Process Red (only if no green pillar was found, giving green priority)
    if detected_color is None:
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours_red:
            largest_contour_red = None
            max_area_red = 0
            for cnt in contours_red:
                area = cv2.contourArea(cnt)
                if area > MIN_PILLAR_AREA:
                    if area > max_area_red:
                        max_area_red = area
                        largest_contour_red = cnt
            
            if largest_contour_red is not None:
                M = cv2.moments(largest_contour_red)
                if M["m00"] != 0:
                    center_x = int(M["m10"] / M["m00"])
                    detected_color = 'red'

                    # Draw bounding box and text for debugging in main.py (optional, handled in main.py)
                    # x, y, w, h = cv2.boundingRect(largest_contour_red)
                    # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                    # cv2.putText(frame, "Red", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                    return detected_color, center_x

    # If no valid green or red pillar was detected
    return None, None

# This __name__ == "__main__" block is for testing this file independently
if __name__ == "__main__":
    print("--- Testing obstacle_avoid.py (Pillar Detection) ---")
    print("Ensure a camera is connected and green/red objects are in view.")
    print("Press 'q' to quit the camera feed.")

    camera_index = 0
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print(f"ERROR: Could not open camera at index {camera_index}.")
    else:
        # Set resolution for testing
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        print(f"Camera opened: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("Failed to grab frame.")
                    break

                color, cx = detect_pillar_color_and_position(frame)

                # --- Visual Debugging for this test script ---
                if color:
                    draw_color = (0, 255, 0) if color == 'green' else (0, 0, 255)
                    text = f"{color.capitalize()} Pillar @ X:{int(cx)}" if cx is not None else f"{color.capitalize()} Pillar"
                    if cx is not None:
                        # Draw a small circle at the detected center X
                        cv2.circle(frame, (cx, frame.shape[0] // 2), 5, draw_color, -1)
                        
                        # Draw a rectangle around the estimated pillar area (example size)
                        box_width = 80
                        box_height = 120
                        x1 = max(0, int(cx - box_width / 2))
                        y1 = max(0, int(frame.shape[0] / 2 - box_height / 2))
                        x2 = min(frame.shape[1], int(cx + box_width / 2))
                        y2 = min(frame.shape[0], int(frame.shape[0] / 2 + box_height / 2))
                        cv2.rectangle(frame, (x1, y1), (x2, y2), draw_color, 2)

                    cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, draw_color, 2)
                else:
                    cv2.putText(frame, "No Pillar Detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                cv2.imshow("Pillar Detection Test", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        except Exception as e:
            print(f"An error occurred during test: {e}")
        finally:
            cap.release()
            cv2.destroyAllWindows()
            print("Pillar detection test finished.")