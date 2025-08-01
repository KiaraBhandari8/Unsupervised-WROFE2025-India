import io
import threading
import time
import cv2
import numpy as np
from flask import Flask, Response, render_template_string

from picamera2 import Picamera2
import robot_motion_test
from process_frames_test import get_robot_direction_and_angle
import libcamera

app = Flask(__name__)

# Shared state variables
latest_raw_jpeg = None
latest_viz_jpeg = None
latest_angle = 0
latest_final_steering_angle = 75
latest_command = "STOP"
lock = threading.Lock()

# Set True if robot facing original direction, False if flipped
facing_original_direction = False

def camera_loop():
    global latest_raw_jpeg, latest_viz_jpeg, latest_angle, latest_final_steering_angle, latest_command

    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (768, 432)},
        transform=libcamera.Transform(vflip=True, hflip=True)  # Flip here if needed for camera orientation
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(2)  # Camera warm-up

    print("Camera loop started")
    while True:
        try:
            frame = picam2.capture_array()

            # Get movement command and steering angle, accounting for robot orientation
            command, steering_angle, viz_frame = get_robot_direction_and_angle(frame, facing_original_direction)

            if steering_angle is None:
                robot_motion_test.robot_stop()
                final_steering_angle = 75  # Neutral servo position
            else:
                # Map steering angle to servo value (adjust sign if servo direction is reversed)
                final_steering_angle = int(75 + (-1 * steering_angle))
                print(f"Steering Angle: {steering_angle}, Final Angle: {final_steering_angle}")

            # Execute robot motion based on command
            if command in ("FORWARD", "LEFT", "RIGHT"):
                robot_motion_test.robot_forward()
                robot_motion_test.adjust_servo_angle(final_steering_angle)
            else:
                robot_motion_test.robot_stop()

            # Encode images for streaming
            _, raw_jpeg = cv2.imencode('.jpg', frame)
            _, viz_jpeg = cv2.imencode('.jpg', viz_frame if viz_frame is not None else frame)

            with lock:
                latest_raw_jpeg = raw_jpeg.tobytes()
                latest_viz_jpeg = viz_jpeg.tobytes()
                latest_angle = steering_angle if steering_angle is not None else 0
                latest_final_steering_angle = final_steering_angle
                latest_command = command

            time.sleep(0.1)

        except Exception as e:
            print(f"Error in camera loop: {e}")
            time.sleep(1)  # Wait a bit before retrying on error

HTML_PAGE = """
<!doctype html>
<title>Robot Stream</title>
<body>
  <h2>Raw Feed</h2>
  <img src="/raw_stream">
  <h2>Processed Feed</h2>
  <img src="/viz_stream">
  <h3>Command: {{ command }}</h3>
  <h3>Steering Angle: {{ angle }}</h3>
  <h3>Final Steering Angle: {{ fangle }}</h3>
  <meta http-equiv="refresh" content="1">
</body>
"""

@app.route('/')
def index():
    with lock:
        return render_template_string(
            HTML_PAGE,
            command=latest_command,
            angle=latest_angle,
            fangle=latest_final_steering_angle
        )

@app.route('/raw_stream')
def raw_stream():
    return Response(gen_image_stream('raw'), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/viz_stream')
def viz_stream():
    return Response(gen_image_stream('viz'), mimetype='multipart/x-mixed-replace; boundary=frame')

def gen_image_stream(stream_type):
    while True:
        with lock:
            frame = latest_raw_jpeg if stream_type == 'raw' else latest_viz_jpeg
        if frame:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.1)

if __name__ == "__main__":
    threading.Thread(target=camera_loop, daemon=True).start()
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
