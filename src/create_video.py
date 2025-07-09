import cv2
import os

def create_video_from_images(image_folder, output_video_name="output_video.mp4", fps=30, image_size=None):
    """
    Creates a video from a folder containing image files.

    Args:
        image_folder (str): The path to the directory containing the images.
        output_video_name (str): The name of the output video file (e.g., "my_video.mp4").
        fps (int): Frames per second for the output video.
        image_size (tuple, optional): A tuple (width, height) for the desired video frame size.
                                      If None, it will use the size of the first image found.
                                      All images will be resized to this dimension if provided.
    """
    # Get all image files from the folder
    images = [img for img in os.listdir(image_folder) if img.endswith((".png", ".jpg", ".jpeg", ".bmp", ".gif"))]
    # Sort the images to ensure they are in the correct order for the video
    images.sort()

    if not images:
        print(f"No images found in the folder: {image_folder}")
        return

    # Determine the frame size
    if image_size is None:
        # Read the first image to get its dimensions
        first_image_path = os.path.join(image_folder, images[0])
        try:
            frame = cv2.imread(first_image_path)
            if frame is None:
                print(f"Could not read the first image: {first_image_path}. Please check the file path and format.")
                return
            height, width, layers = frame.shape
            image_size = (width, height)
            print(f"Using image size from first image: {image_size}")
        except Exception as e:
            print(f"Error reading first image: {e}")
            return
    else:
        print(f"Using specified image size: {image_size}")

    # Define the codec and create VideoWriter object
    # For MP4, 'mp4v' or 'XVID' often works. 'avc1' for H.264 (more common but might need specific codecs installed)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v') # You can try 'XVID' or 'MJPG' if 'mp4v' doesn't work
    video = cv2.VideoWriter(output_video_name, fourcc, fps, image_size)

    if not video.isOpened():
        print(f"Error: Could not open video writer for {output_video_name}. Check codec or file path.")
        return

    print(f"Starting video creation with {len(images)} images...")
    for i, image_name in enumerate(images):
        image_path = os.path.join(image_folder, image_name)
        try:
            frame = cv2.imread(image_path)
            if frame is None:
                print(f"Warning: Could not read image {image_path}. Skipping.")
                continue

            # Resize frame if a specific image_size is provided and it differs
            if frame.shape[1] != image_size[0] or frame.shape[0] != image_size[1]:
                frame = cv2.resize(frame, image_size)

            video.write(frame)
            if (i + 1) % 10 == 0: # Print progress every 10 images
                print(f"Processed {i + 1}/{len(images)} images...")
        except Exception as e:
            print(f"Error processing image {image_path}: {e}")
            continue

    video.release()
    print(f"Video '{output_video_name}' created successfully in the current directory!")

# --- Example Usage ---
if __name__ == "__main__":
    # IMPORTANT: Replace 'path/to/your/images' with the actual path to your image directory
    # For example, if your images are in a folder named 'my_photos' in the same directory as this script:
    # image_directory = 'my_photos'
    image_directory = 'capture_images' # <--- CHANGE THIS TO YOUR IMAGE FOLDER PATH

    # Ensure the directory exists for the example to run
    if not os.path.exists(image_directory):
        print(f"Error: The directory '{image_directory}' does not exist.")
        print("Please create this directory and place some images inside, or update the 'image_directory' variable.")
        # You can create a dummy directory and images for testing:
        # os.makedirs(image_directory, exist_ok=True)
        # from PIL import Image
        # for i in range(10):
        #     img = Image.new('RGB', (640, 480), color = (i*20, i*10, 255-i*20))
        #     img.save(os.path.join(image_directory, f'image_{i:03d}.png'))
        # print(f"Created dummy images in '{image_directory}' for testing.")
        # create_video_from_images(image_directory, "my_test_video.mp4", fps=15, image_size=(640, 480))
    else:
        # Call the function to create the video
        # You can adjust the output name, frames per second (fps), and image_size as needed.
        create_video_from_images(
            image_folder=image_directory,
            output_video_name="my_output_video.mp4",
            fps=24, # Common frame rates are 24, 25, 30
            # image_size=(1920, 1080) # Uncomment and set if you want a specific resolution (width, height)
        )
