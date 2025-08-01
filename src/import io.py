import io
import threading
import time
import cv2
import numpy as np
from flask import Flask, Response, render_template_string, jsonify

# It's assumed you have these modules from your project
from picamera2 import Picamera2
from process_frames import get_robot_direction_and_angle
import robot_motion
import libcamera

# --- Flask App Setup ---
app = Flask(__name__)

# --- Shared Data with Threading Lock ---
lock = threading.Lock()
latest_raw_jpeg = None
latest_viz_jpeg = None
latest_angle = 0
latest_final_steering_angle = 0
latest_command = "STOP"
facing_original_direction = True

# --- Steering Smoothing Variable ---
smoothed_final_angle = 75.0

def camera_loop():
    """
    This function runs in a separate thread. It continuously captures frames
    from the camera, processes them, controls the robot, and updates the
    shared variables for streaming.
    """
    global latest_raw_jpeg, latest_viz_jpeg, latest_angle, latest_final_steering_angle, latest_command
    global smoothed_final_angle

    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(
        main={"size": (1280, 720)},
        transform=libcamera.Transform(vflip=True, hflip=True)
    )
    picam2.configure(camera_config)
    picam2.start()
    time.sleep(2.0)

    while True:
        frame_array = picam2.capture_array()
        command, steering_angle, viz_frame = get_robot_direction_and_angle(frame_array)

        current_servo_angle = 75
        if steering_angle is None:
            robot_motion.robot_stop()
        else:
            steering_angle = int(steering_angle)
            if not facing_original_direction:
                steering_angle = -1 * steering_angle

            if steering_angle > 0:
                steering_angle = steering_angle * 3

            target_servo_angle = int(75 + (-1 * steering_angle))
            
            smoothing_factor = 0.4
            smoothed_final_angle = (target_servo_angle * smoothing_factor) + (smoothed_final_angle * (1.0 - smoothing_factor))
            
            current_servo_angle = int(smoothed_final_angle)

            if command in ("FORWARD", "LEFT", "RIGHT"):
                robot_motion.robot_forward()
                robot_motion.adjust_servo_angle(current_servo_angle)
            else:
                robot_motion.robot_stop()

        output_viz_frame = viz_frame if viz_frame is not None else frame_array
        _, viz_jpeg_encoded = cv2.imencode('.jpg', output_viz_frame)
        raw_jpeg_bytes = picam2.capture_jpeg()

        with lock:
            latest_raw_jpeg = raw_jpeg_bytes
            latest_viz_jpeg = viz_jpeg_encoded.tobytes()
            latest_angle = steering_angle if steering_angle is not None else 0
            latest_final_steering_angle = current_servo_angle
            latest_command = command

        time.sleep(0.05)

# --- Flask Web Server Routes ---

HTML_PAGE = """
<!doctype html>
<html>
<head>
  <title>Robot Camera Stream</title>
  <style>
    body { background-color: #2c3e50; color: #ecf0f1; font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; display: flex; flex-direction: column; align-items: center; margin: 0; padding: 20px; }
    .header-info { text-align: center; margin-bottom: 20px; padding: 10px; background-color: #34495e; border-radius: 8px; width: 90%; max-width: 980px; }
    .header-info p { margin: 5px 0; font-size: 0.9em; color: #bdc3c7; }
    .content-wrapper { display: flex; flex-direction: row; flex-wrap: wrap; gap: 30px; justify-content: center; }
    .frame-group { display: flex; flex-direction: column; align-items: center; background-color: #34495e; padding: 20px; border-radius: 12px; box-shadow: 0 4px 15px rgba(0,0,0,0.2); }
    .info-box { text-align: center; margin-bottom: 15px; }
    h2 { margin-top: 0; font-size: 1.5em; color: #3498db; }
    h4 { margin: 8px 0; font-size: 1.1em; }
    h4 span { font-weight: bold; color: #e67e22; padding-left: 8px; }
    img { border: 2px solid #3498db; border-radius: 8px; max-width: 100%; width: 480px; height: auto; display: block; background-color: #000; }
  </style>
</head>
<body>
  <!-- New Header Section -->
  <div class="header-info">
    <p><strong>Location:</strong> Mumbai, Maharashtra, India</p>
    <p><strong>Time:</strong> Thursday, July 10, 2025 at 6:23 PM IST</p>
  </div>

  <div class="content-wrapper">
    <div class="frame-group">
      <div class="info-box">
        <h2>Raw Camera Feed</h2>
      </div>
      <img src="{{ url_for('raw_stream') }}" alt="Raw Stream">
    </div>

    <div class="frame-group">
      <div class="info-box">
        <h2>Processed Visualization</h2>
        <h4 id="data-command">Command: <span>LOADING...</span></h4>
        <h4 id="data-angle">Raw Steering Angle: <span>LOADING...</span></h4>
        <h4 id="data-fangle">Smoothed Servo Angle: <span>LOADING...</span></h4>
      </div>
      <img src="{{ url_for('viz_stream') }}" alt="Visualization Stream">
    </div>
  </div>

  <script>
    function updateData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          document.getElementById('data-command').querySelector('span').innerText = data.command;
          document.getElementById('data-angle').querySelector('span').innerText = data.angle;
          document.getElementById('data-fangle').querySelector('span').innerText = data.fangle;
        })
        .catch(error => console.error('Error fetching data:', error));
    }
    setInterval(updateData, 500);
  </script>
</body>
</html>
"""

@app.route('/')
def index():
    """Serves the main HTML page."""
    return render_template_string(HTML_PAGE)

@app.route('/data')
def data():
    """Provides robot data as a JSON object."""
    with lock:
        return jsonify(
            angle=latest_angle,
            fangle=latest_final_steering_angle,
            command=latest_command
        )

def gen_image_stream(image_type):
    """A generator function that yields camera frames for streaming."""
    while True:
        time.sleep(0.05)
        with lock:
            if image_type == 'raw':
                frame = latest_raw_jpeg
            else:
                frame = latest_viz_jpeg

        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/raw_stream')
def raw_stream():
    """Route for the raw camera stream."""
    return Response(gen_image_stream('raw'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/viz_stream')
def viz_stream():
    """Route for the processed visualization stream."""
    return Response(gen_image_stream('viz'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    camera_thread = threading.Thread(target=camera_loop, daemon=True)
    camera_thread.start()
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
