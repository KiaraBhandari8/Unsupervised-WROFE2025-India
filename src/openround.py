import cv2
import numpy as np
import matplotlib.pyplot as plt

def filter_blue_objects(hsv_frame):
    """Detects and returns a processed mask for blue objects."""
    lower_blue = np.array([80, 110, 50])
    upper_blue = np.array([130, 255, 255])
    blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
    kernel = np.ones((5, 5), np.uint8)
    blue_mask_eroded = cv2.erode(blue_mask, kernel, iterations=2)
    blue_mask_dilated = cv2.dilate(blue_mask_eroded, kernel, iterations=2)
    blue_contours, _ = cv2.findContours(blue_mask_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return blue_mask_dilated

def filter_orange_objects(hsv_frame):
    """Detects and returns a processed mask for orange objects."""
    lower_orange = np.array([5, 100, 20])
    upper_orange = np.array([15, 255, 255])
    orange_mask = cv2.inRange(hsv_frame, lower_orange, upper_orange)
    kernel = np.ones((5, 5), np.uint8)
    orange_mask_eroded = cv2.erode(orange_mask, kernel, iterations=2)
    orange_mask_dilated = cv2.dilate(orange_mask_eroded, kernel, iterations=2)
    orange_contours, _ = cv2.findContours(orange_mask_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return orange_mask_dilated

# Load the image
image = cv2.imread('line_tracking8.jpg')
if image is None:
    raise FileNotFoundError("line_tracking8.jpg not found. Please make sure the image exists.")

# Convert image to HSV color space
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# Get processed color masks
blue_mask = filter_blue_objects(hsv)
orange_mask = filter_orange_objects(hsv)

# Convert the original image to RGB for Matplotlib
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Create and save the comparison figure
fig, axs = plt.subplots(1, 3, figsize=(15, 5))
axs[0].imshow(image_rgb)
axs[0].set_title('Original Image')
axs[0].axis('off')

axs[1].imshow(blue_mask, cmap='gray')
axs[1].set_title('Blue Filtered Mask')
axs[1].axis('off')

axs[2].imshow(orange_mask, cmap='gray')
axs[2].set_title('Orange Filtered Mask')
axs[2].axis('off')

plt.tight_layout()
plt.savefig("color_filter_comparison.png")
plt.close()
