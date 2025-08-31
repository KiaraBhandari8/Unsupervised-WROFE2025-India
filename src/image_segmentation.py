import cv2
import numpy as np
import os
import datetime

def segment_colors_and_save_combined(image_path):
    """
    Loads an image, performs red and green segmentation, and saves
    a single combined image with original, red-segmented, and green-segmented
    outputs side-by-side, separated by borders.
    """
    # 1. Check if the image file exists
    if not os.path.exists(image_path):
        print(f"Error: The image file '{image_path}' was not found.")
        return

    # 2. Load the image
    frame = cv2.imread(image_path)
    if frame is None:
        print(f"Error: Could not read the image from '{image_path}'.")
        return

    # 3. Convert to HSV for better color segmentation
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 4. Define the color masks
    lower_green = np.array([35, 100, 50])
    upper_green = np.array([85, 255, 255])
    lower_red1 = np.array([0, 150, 100])
    upper_red1 = np.array([7, 255, 255])
    lower_red2 = np.array([173, 150, 100])
    upper_red2 = np.array([180, 255, 255])

    # 5. Create individual masks for green and red
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # 6. Apply each mask to the original image
    green_segmented_image = cv2.bitwise_and(frame, frame, mask=mask_green)
    red_segmented_image = cv2.bitwise_and(frame, frame, mask=mask_red)
    
    # 7. Resize images to a smaller size for efficient saving
    scale_percent = 50
    width = int(frame.shape[1] * scale_percent / 100)
    height = int(frame.shape[0] * scale_percent / 100)
    dim = (width, height)
    
    resized_original = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
    resized_red = cv2.resize(red_segmented_image, dim, interpolation=cv2.INTER_AREA)
    resized_green = cv2.resize(green_segmented_image, dim, interpolation=cv2.INTER_AREA)

    # 8. Combine the three resized images side-by-side
    combined_output_image = np.hstack((resized_original, resized_red, resized_green))

    # --- NEW CODE: ADDING BORDERS ---
    
    # Define border color and thickness
    border_color = (255, 255, 255)  # White color in BGR
    border_thickness = 5           # Thickness in pixels

    # Draw the first vertical line between the original and red-segmented images
    # The x-coordinate is the width of the first image (resized_original)
    cv2.line(combined_output_image, (width, 0), (width, height), border_color, border_thickness)

    # Draw the second vertical line between the red and green-segmented images
    # The x-coordinate is the width of the first two images combined (width * 2)
    cv2.line(combined_output_image, (width * 2, 0), (width * 2, height), border_color, border_thickness)
    
    # --- END OF NEW CODE ---

    # 9. Save the combined output image to a new folder
    input_dir = os.path.dirname(image_path)
    output_folder = os.path.join(input_dir, "segmentation_combined_outputs")
    
    os.makedirs(output_folder, exist_ok=True)
    
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    combined_output_filename = f"combined_segmentation_{timestamp}.jpg"
    full_output_path = os.path.join(output_folder, combined_output_filename)
    
    cv2.imwrite(full_output_path, combined_output_image)
    
    print(f"Saved combined segmentation output to: '{full_output_path}'")
    
# --- Main part of the script ---
if __name__ == "__main__":
    image_to_process = "/home/pi8/wrofe2025/D:\Captured_images/segment_2025-08-31_17-00-24.jpg"
    
    segment_colors_and_save_combined(image_to_process)