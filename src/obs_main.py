import cv2
import sys
import numpy as np
from picamera2 import Picamera2
import libcamera
from flask import Flask, render_template, Response
import threading
import time

# Import your custom functions
try:
    # I recommend renaming your vision file to "image_processing.py" for clarity
    from image_frame import process_frame_for_steering
    from robot_motion import adjust_servo_angle, robot_forward_speed, robot_stop
except ImportError as e:
    print(f"Error importing modules: {e}")
    print("Please ensure 'image_processing.py' and 'robot_motion.py' are in the same directory.")
    sys.exit()

# --- Frame Server & Threading Setup ---
# This global variable will hold the latest frame to be streamed
output_frame = None
# A lock is used to prevent data races between threads
lock = threading.Lock()

# --- Flask App Initialization ---
app = Flask(__name__)

# --- Robot Control Parameters ---
SERVO_CENTER_ANGLE = 90
STEERING_GAIN = 0.1
ROBOT_SPEED = 0.5

def robot_control_loop():
    """
    Main loop for robot control, runs in a separate thread.
    Captures frames, processes them, commands the robot, and updates
    the global 'output_frame' for web streaming.
    """
    global output_frame, lock

    # --- Picamera2 Initialization ---
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(
        main={"size": (768, 432)},
        transform=libcamera.Transform(vflip=True, hflip=True)
    )
    picam2.configure(camera_config)
    picam2.start()
    print("Robot control loop started... Press Ctrl+C in the terminal to stop the web server.")

    try:
        while True:
            # 1. CAPTURE & PROCESS FRAME
            frame_rgb = picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            # Get the processed frame and steering angle
            processed_frame, steering_angle, _ = process_frame_for_steering(frame_bgr)

            # 2. CALCULATE SERVO ANGLE
            servo_adjustment = -1 * steering_angle * STEERING_GAIN
            target_servo_angle = SERVO_CENTER_ANGLE - servo_adjustment
            # Clamp the angle to the desired range (60 to 120 degrees)
            target_servo_angle = np.clip(target_servo_angle, 60, 120)

            # 3. COMMAND THE ROBOT
            adjust_servo_angle(target_servo_angle)
            robot_forward_speed(ROBOT_SPEED)

            # Optional: Print status to the console
            # print(f"Vision Angle: {steering_angle:.2f}, Servo Target: {target_servo_angle:.2f}")

            # 4. PREPARE FRAME FOR STREAMING
            # Add steering info text to the frame
            info_text = f"Servo Angle: {target_servo_angle:.2f}"
            cv2.putText(processed_frame, info_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            # Update the global output frame safely
            with lock:
                output_frame = processed_frame.copy()

    finally:
        # Cleanly stop the robot and camera on exit.
        robot_stop()
        picam2.stop()
        print("Robot and camera stopped.")

def generate_frames():
    """Generator function for the Flask video stream."""
    global output_frame, lock
    while True:
        with lock:
            if output_frame is None:
                time.sleep(0.01) # Wait if no frame is available yet
                continue
            
            # Encode the frame in JPEG format
            (flag, encoded_image) = cv2.imencode(".jpg", output_frame)
            if not flag:
                continue

        # Yield the output frame in the byte format for streaming
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
              bytearray(encoded_image) + b'\r\n')

@app.route("/")
def index():
    """Video streaming home page."""
    # This renders the index.html file from the 'templates' folder
    return render_template("index.html")

@app.route("/video_feed")
def video_feed():
    """Video streaming route. This is the endpoint for the <img> tag."""
    return Response(generate_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == '__main__':
    # Start the robot control loop in a background thread
    control_thread = threading.Thread(target=robot_control_loop)
    control_thread.daemon = True # Allows main thread to exit even if this thread is running
    control_thread.start()
    
    # Start the Flask web server
    print("Starting web server... Open http://<your_pi_ip>:5000 in a browser.")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)
