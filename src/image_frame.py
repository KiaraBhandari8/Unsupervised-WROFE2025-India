import cv2
import numpy as np
import sys

# --- Control Parameters ---
# Proportional gain for the steering angle. Tune this for your robot.
KP_STEERING = 0.5

def process_frame_for_steering(frame):
    """
    Processes a single color frame to detect the bottom edge of a green obstacle,
    calculate a steering angle, and draw visualizations.

    Args:
        frame: The input color image (as a NumPy array).

    Returns:
        A tuple containing:
        - processed_frame: The original frame with UI and detection visuals drawn on it.
        - steering_angle: The calculated steering angle (float).
        - binary_frame: The binary mask used for detection.
    """
    if frame is None:
        return None, 0, None

    # --- Convert Original Frame to Binary based on Green Color ---
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_green = np.array([35, 100, 50])
    upper_green = np.array([85, 255, 255])
    binary_frame = cv2.inRange(hsv_frame, lower_green, upper_green)

    # Get frame dimensions
    h, w = frame.shape[:2]

    # --- Define UI Colors and Thickness ---
    ui_color = (0, 255, 0)
    obstacle_color = (0, 0, 255)
    thickness = 2

    # --- Calculate UI Coordinates ---
    outer_rect_start = (int(0.15 * w), int(0.15 * h))
    outer_rect_end = (int(0.85 * w), int(0.85 * h))
    inner_rect_start = (int(0.15 * w), int(0.25 * h))
    inner_rect_end = (int(0.85 * w), int(0.75 * h))
    line_start = (int(0.5 * w), int(0.25 * h))
    line_end = (int(0.5 * w), int(0.75 * h))
    
    # Create a copy of the frame to draw on
    processed_frame = frame.copy()
    steering_angle = 0

    # --- Obstacle Detection and Steering Calculation ---
    contours, _ = cv2.findContours(binary_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Find the largest contour by area
        largest_contour = max(contours, key=cv2.contourArea)

        # --- NEW LOGIC: Find the bottom-most point of the contour ---
        # The y-coordinate is at index 1. argmax() finds the index of the max y-value.
        bottom_most_point = tuple(largest_contour[largest_contour[:, :, 1].argmax()][0])
        pX, pY = bottom_most_point
        # --- END OF NEW LOGIC ---

        # Check if the detected bottom point is inside the OUTER rectangle
        if outer_rect_start[0] < pX < outer_rect_end[0] and \
           outer_rect_start[1] < pY < outer_rect_end[1]:
            
            # The target is the x-coordinate of the right edge of the inner rectangle
            target_x = inner_rect_end[0]
            # Calculate the error between the target and the detected point's x-position
            error = target_x - pX
            # Calculate the steering angle using the proportional gain
            steering_angle = KP_STEERING * error

            # Draw visualization on the processed frame using the bottom-most point
            cv2.drawContours(processed_frame, [largest_contour], -1, obstacle_color, thickness)
            # Draw a circle at the detected bottom point
            cv2.circle(processed_frame, (pX, pY), 7, obstacle_color, -1)
            # Draw a line from the bottom point to the target edge
            cv2.line(processed_frame, (pX, pY), (target_x, pY), (255, 0, 0), thickness)

    # --- Draw the UI on the Frame ---
    cv2.rectangle(processed_frame, outer_rect_start, outer_rect_end, ui_color, thickness)
    cv2.rectangle(processed_frame, inner_rect_start, inner_rect_end, ui_color, thickness)
    cv2.line(processed_frame, line_start, line_end, ui_color, thickness)
    cv2.line(processed_frame, (inner_rect_end[0], inner_rect_start[1]), inner_rect_end, (255, 255, 0), thickness)
    
    return processed_frame, steering_angle, binary_frame

# --- Main Execution Block (for testing the function with a static image) ---
if __name__ == '__main__':
    # Load the image
    image_filename = 'obs_frame.jpg'
    original_frame = cv2.imread(image_filename)

    # Error handling for image loading
    if original_frame is None:
        print(f"Error: Could not read the image file '{image_filename}'.")
        sys.exit()

    # Process the frame using the function
    final_frame, angle, binary_mask = process_frame_for_steering(original_frame)
    
    if final_frame is not None:
        # --- Display Steering Information ---
        info_text = f"Steering Angle: {angle:.2f}"
        cv2.putText(final_frame, info_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        print(info_text)

        # --- Save and Display the Result ---
        output_filename = 'obs_frame_with_overlay.jpg'
        cv2.imwrite(output_filename, final_frame)
        print(f"Overlay complete. Image saved as '{output_filename}'")

        cv2.imshow("Image with Overlay", final_frame)
        if binary_mask is not None:
            cv2.imshow("Binary Mask", binary_mask)
        
        print("Press any key to close the image windows.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
