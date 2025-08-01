# main1.py or main_test.py
import io
import threading
import time
import cv2
import numpy as np
from flask import Flask, Response, render_template_string

from picamera2 import Picamera2
from process_frames_test import get_robot_direction_and_angle
import robot_motion_test
import libcamera

app = Flask(__name__)

latest_raw_jpeg = None
latest_viz_jpeg = None
latest_angle = 0
latest_final_steering_angle = 0
latest_command = "STOP"
latest_angle_area = 0
latest_angle_centroid = 0
lock = threading.Lock()

facing_original_direction = True  # Set this to False if the robot is reversed

def camera_loop():
    global latest_raw_jpeg, latest_viz_jpeg, latest_angle, latest_final_steering_angle, latest_command, latest_angle_centroid, latest_angle_area

    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(main={"size": (768, 432)},
        transform=libcamera.Transform(vflip=True, hflip=True))
    picam2.configure(camera_config)
    picam2.start()
    time.sleep(2)

    while True:
        frame = picam2.capture_array()
        command, steering_angle, viz_frame= get_robot_direction_and_angle(
            frame, facing_original_direction
        )

        if steering_angle is None:
            robot_motion_test.robot_stop()
            final_steering_angle = 75
        else:
            steering_angle = int(steering_angle)

            # Flip logic for reversed direction if needed
            if not facing_original_direction:
                steering_angle = -steering_angle

            print(f"Weighted Steering Angle : {steering_angle}")
            final_steering_angle = int(75 + (-1 * steering_angle))
            print(f"Final Steering Angle : {final_steering_angle}")

        if command in ("FORWARD", "LEFT", "RIGHT"):
            robot_motion_test.robot_forward()
            robot_motion_test.adjust_servo_angle(final_steering_angle)
        else:
            robot_motion_test.robot_stop()

        _, raw_jpeg = cv2.imencode('.jpg', frame)
        _, viz_jpeg = cv2.imencode('.jpg', viz_frame if viz_frame is not None else frame)

        with lock:
            latest_raw_jpeg = raw_jpeg.tobytes()
            latest_viz_jpeg = viz_jpeg.tobytes()
            latest_angle = steering_angle if steering_angle is not None else 0
            latest_final_steering_angle = final_steering_angle
            latest_command = command
            latest_angle_area = angle_area
            latest_angle_centroid = angle_centroid

        time.sleep(0.1)

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
    /* Add padding to the top of the raw camera feed's info-box 
       to visually match the height of the processed info-box */
  }
  .info-box {
     /* Space between info and image */
    text-align: center;
    /* You could try setting a min-height for info-box here if needed */
  }
  .first-box{
  margin-bottom: 30px; 
  }
  .second-box{
  margin-bottom: 90px; 
  }
  .info-box.raw-feed-info {
    /* Calculate the approximate difference in height and add padding */
    /* This value might need tweaking based on font size and browser rendering */
    padding-bottom: 50px; /* Adjust this value as needed */
    min-height: 100px; /* Example: set a minimum height to match the other side */
    display: flex; /* Make it a flex container to center its content */
    flex-direction: column;
    margin-bottom: 10px;
    justify-content: center; /* Vertically center content if min-height is larger */
  }
  h2, h3, h4 {
      margin: 0; /* Remove default margins from headers to reduce extra space */
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
      <div class="info-box first-box raw-feed-info">
        <h2>Raw Camera Feed</h2>
      </div>
      <img src="/raw_stream">
    </div>

    <div class="frame-group">
      <div class="info-box second-box">
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
        angle_centroid = latest_angle_centroid
        angle_area = latest_angle_area
    return render_template_string(HTML_PAGE, angle=angle, fangle=fangle, command=command,
                                  angle_centroid=angle_centroid, angle_area=angle_area)

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

if __name__ == "__main__":
    t = threading.Thread(target=camera_loop, daemon=True)
    t.start()
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
