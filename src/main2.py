import io
import threading
import time
import cv2
import numpy as np
from flask import Flask, Response, render_template_string

from picamera2 import Picamera2
from process_frames import get_robot_direction_and_angle
import robot_motion
import libcamera

# --- Flask App Setup ---
app = Flask(__name__)

# Shared data for streaming
latest_raw_jpeg = None
latest_viz_jpeg = None
latest_angle = 0
latest_final_steering_angle = 0
latest_command = "STOP"
lock = threading.Lock()
facing_original_direction = True  # CHANGE THIS as needed

# --- Video Recording Settings ---
record_video = True  # Set to True to enable recording
output_combined_video_path = "combined_robot_feed.avi" # New combined video output
video_codec = cv2.VideoWriter_fourcc(*'MJPG') # Or 'XVID', 'DIVX' for compatibility
fps = 10 # Frames per second for recorded video (adjust as needed)

# VideoWriter object (initialized later if recording is enabled)
combined_video_writer = None

def camera_loop():
    global latest_raw_jpeg, latest_viz_jpeg, latest_angle, latest_final_steering_angle, latest_command
    global combined_video_writer
    global record_video

    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(
        main={"size": (768, 432), "format": "XRGB8888"}, # Request 4-channel format for direct capture
        transform=libcamera.Transform(vflip=True, hflip=True)
    )
    picam2.configure(camera_config)
    picam2.start()
    time.sleep(2)  # Camera warm-up

    # Get frame dimensions for video writer initialization
    initial_frame = picam2.capture_array()
    if initial_frame.shape[2] == 4:
        initial_frame = cv2.cvtColor(initial_frame, cv2.COLOR_RGBA2BGR)
    
    h, w, _ = initial_frame.shape

    if record_video:
        try:
            combined_video_writer = cv2.VideoWriter(output_combined_video_path, video_codec, fps, (2 * w, h))
            if not combined_video_writer.isOpened():
                print("Error: Could not open combined video writer.")
                record_video = False
            else:
                print(f"Recording combined video to {output_combined_video_path}")
        except Exception as e:
            print(f"Error initializing combined video writer: {e}")
            record_video = False

    while True:
        frame = picam2.capture_array()
        
        # --- Convert raw frame to 3-channel BGR ---
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

        # Call get_robot_direction_and_angle with only the frame
        command, steering_angle_unmapped, viz_frame = get_robot_direction_and_angle(frame)
        
        # --- Handle potential None viz_frame and ensure 3 channels ---
        if viz_frame is None:
            viz_frame = frame.copy() # Fallback to raw frame if processing failed to return a viz_frame
        elif len(viz_frame.shape) < 3 or viz_frame.shape[2] == 4: # Check if it's 2D or 4-channel
            if len(viz_frame.shape) < 3: # If 2D (grayscale), convert to 3-channel BGR
                viz_frame = cv2.cvtColor(viz_frame, cv2.COLOR_GRAY2BGR)
            elif viz_frame.shape[2] == 4: # If 4-channel, convert to 3-channel BGR
                viz_frame = cv2.cvtColor(viz_frame, cv2.COLOR_RGBA2BGR)


        # Initialize current_final_steering_angle for this iteration
        current_final_steering_angle = latest_final_steering_angle

        # Apply robot motion logic and calculate the NEW final_steering_angle
        if steering_angle_unmapped is None:
            robot_motion.robot_stop()
        else:
            steering_angle_unmapped = int(steering_angle_unmapped)
            
            if not facing_original_direction:
                steering_angle_unmapped = -1 * steering_angle_unmapped

            current_final_steering_angle = int(75 + (-1 * int(steering_angle_unmapped)))
            if current_final_steering_angle < 70:
                current_final_steering_angle -= 20
            print(f"Final Steering Angle : {int(current_final_steering_angle)}")

            if command in ("FORWARD", "LEFT", "RIGHT"):
                robot_motion.robot_forward()
                robot_motion.adjust_servo_angle(current_final_steering_angle)
            else:
                robot_motion.robot_stop()
        
        # --- Draw final_steering_angle on viz_frame in main.py ---
        # Ensure viz_frame is not None before drawing
        if viz_frame is not None:
            text = f"Final Angle: {current_final_steering_angle:.2f}"
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1
            font_thickness = 2
            text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
            text_x = viz_frame.shape[1] - text_size[0] - 10
            text_y = text_size[1] + 10
            cv2.putText(viz_frame, text, (text_x, text_y), font, font_scale, (0, 255, 255), font_thickness, cv2.LINE_AA)
        

        # --- Combine frames horizontally for recording and streaming ---
        # Ensure both frames have the same dimensions and are 3-channel BGR
        if frame.shape == viz_frame.shape:
            combined_frame = cv2.hconcat([frame, viz_frame])
        else:
            print(f"CRITICAL WARNING: Frame dimensions still mismatch ({frame.shape} vs {viz_frame.shape}). Recording raw frame only.")
            combined_frame = frame.copy()

        # Encode images for streaming (for web UI)
        _, raw_jpeg = cv2.imencode('.jpg', frame)
        _, viz_jpeg = cv2.imencode('.jpg', viz_frame)


        # Write combined frame to video file if recording is enabled
        if record_video and combined_video_writer is not None and combined_video_writer.isOpened():
            try:
                combined_video_writer.write(combined_frame)
            except Exception as e:
                print(f"Error writing combined frame to video: {e}")


        with lock:
            latest_raw_jpeg = raw_jpeg.tobytes()
            latest_viz_jpeg = viz_jpeg.tobytes()
            latest_angle = steering_angle_unmapped
            latest_final_steering_angle = current_final_steering_angle
            latest_command = command

        time.sleep(0.1)

# --- Flask Routes ---
HTML_PAGE = """
<!doctype html>
<title>Robot Camera Stream</title>
<style>
  body {
    font-family: sans-serif;
    display: flex;
    justify-content: center;
    align-items: flex-start; /* Align content to the top of the body */
    min-height: 100vh;
    margin: 0;
    padding: 20px;
    box-sizing: border-box;
  }
  .content-wrapper {
    display: flex;
    flex-direction: row;
    align-items: flex-start; /* Aligns the tops of the frame-groups */
    flex-wrap: wrap;
    gap: 20px;
  }
  .frame-group {
    display: flex;
    flex-direction: column; /* Stack title/text above the image */
    align-items: center; /* Center content within each group */
  }
  .info-box {
    text-align: center;
  }
  .info-box.raw-feed-info {
    /* Adjust these values to vertically align the images */
    padding-bottom: 50px; /* Adjust as needed, e.g., to match the height of other info-box content */
    min-height: 100px;    /* Adjust as needed, example value */
    display: flex;
    flex-direction: column;
    justify-content: center;
  }
  h2, h3, h4 {
      margin: 0;
  }
  img {
    border: 1px solid #ccc;
    max-width: 480px;
    height: auto;
    display: block;
  }
</style>
<body>
  <div class="content-wrapper">
    <div class="frame-group">
      <div class="info-box raw-feed-info">
        <h2>Raw Camera Feed</h2>
      </div>
      <img src="/raw_stream">
    </div>

    <div class="frame-group">
      <div class="info-box">
        <h2>Processed Visualization</h2>
        <h4>Final Steering Angle: {{ fangle }}</h4>
        <h4>Steering Angle: {{ angle }}</h4>
        <h4 >Command: {{ command }}</h4>
      </div>
      <img src="/viz_stream">
    </div>
  </div>
  <meta http-equiv="refresh" content="1">
</body>
"""

@app.route('/')
def index():
    with lock:
        angle = latest_angle
        fangle = latest_final_steering_angle
        command = latest_command
    return render_template_string(HTML_PAGE, angle=angle, fangle=fangle, command=command)

def gen_image_stream(image_type):
    while True:
        with lock:
            frame = latest_raw_jpeg if image_type == 'raw' else latest_viz_jpeg
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.1)

@app.route('/raw_stream')
def raw_stream():
    return Response(gen_image_stream('raw'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/viz_stream')
def viz_stream():
    return Response(gen_image_stream('viz'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# --- Main Entry Point ---
if __name__ == "__main__":
    t = threading.Thread(target=camera_loop, daemon=True)
    t.start()
    
    try:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("\nStopping application gracefully...")
    finally:
        if combined_video_writer is not None and combined_video_writer.isOpened():
            combined_video_writer.release()
            print(f"Combined video saved to {output_combined_video_path}")
        robot_motion.robot_stop()
        print("Cleanup complete. Exiting.")