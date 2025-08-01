import threading
import time
import cv2
from flask import Flask, Response, render_template_string

from picamera2 import Picamera2
import processframes3
import robot_motion
import libcamera

app = Flask(__name__)

latest_raw_jpeg = None
latest_mask_jpeg = None
latest_viz_jpeg = None
latest_angle = 0
latest_final_steering_angle = 0
latest_command = "STOP"
lock = threading.Lock()

# ... (imports and Flask setup as before)
def camera_loop():
    global latest_raw_jpeg, latest_mask_jpeg, latest_viz_jpeg, latest_angle, latest_final_steering_angle, latest_command
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(main={"size": (768, 432)},
                                                       transform=libcamera.Transform(vflip=True, hflip=True))
    picam2.configure(camera_config)
    picam2.start()
    time.sleep(2)

    while True:
        frame = picam2.capture_array()
        command, steering_angle, viz_frame, mask_frame = processframes3.get_robot_direction_and_angle(frame)
        if steering_angle is None:
            robot_motion.robot_stop()
            final_steering_angle = 75
        else:
            steering_angle = int(steering_angle)
            if steering_angle > 0:
                steering_angle = steering_angle * 2
            final_steering_angle = int(75 + (-1 * int(steering_angle)))

        if command in ("FORWARD", "LEFT", "RIGHT"):
            robot_motion.robot_forward()
            robot_motion.adjust_servo_angle(final_steering_angle)
        else:
            robot_motion.robot_stop()

        _, raw_jpeg = cv2.imencode('.jpg', frame)
        _, mask_jpeg = cv2.imencode('.jpg', mask_frame if mask_frame is not None else frame)
        _, viz_jpeg = cv2.imencode('.jpg', viz_frame if viz_frame is not None else frame)

        with lock:
            latest_raw_jpeg = raw_jpeg.tobytes()
            latest_mask_jpeg = mask_jpeg.tobytes()
            latest_viz_jpeg = viz_jpeg.tobytes()
            latest_angle = steering_angle if steering_angle is not None else 0
            latest_final_steering_angle = final_steering_angle
            latest_command = command

        time.sleep(0.1)
# ... (Flask routes and main entry as before)


HTML_PAGE = """
<!doctype html>
<title>Robot Camera Stream</title>
<style>
  body { font-family: sans-serif; display: flex; justify-content: center; align-items: flex-start; min-height: 100vh; margin: 0; padding: 20px; box-sizing: border-box;}
  .content-wrapper { display: flex; flex-direction: row; align-items: flex-start; flex-wrap: wrap; gap: 20px; }
  .frame-group { display: flex; flex-direction: column; align-items: center; }
  .info-box { text-align: center; }
  img { border: 1px solid #ccc; max-width: 420px; height: auto; display: block; }
</style>
<body>
  <div class="content-wrapper">
    <div class="frame-group">
      <div class="info-box">
        <h2>Raw Camera Feed</h2>
      </div>
      <img src="/raw_stream">
    </div>
    <div class="frame-group">
      <div class="info-box">
        <h2>Driveable Mask</h2>
      </div>
      <img src="/mask_stream">
    </div>
    <div class="frame-group">
      <div class="info-box">
        <h2>Final Visualization</h2>
        <h4>Final Steering Angle: {{ fangle }}</h4>
        <h4>Steering Angle: {{ angle }}</h4>
        <h4>Command: {{ command }}</h4>
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
            if image_type == 'raw':
                frame = latest_raw_jpeg
            elif image_type == 'mask':
                frame = latest_mask_jpeg
            else:
                frame = latest_viz_jpeg
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        time.sleep(0.1)

@app.route('/raw_stream')
def raw_stream():
    return Response(gen_image_stream('raw'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/mask_stream')
def mask_stream():
    return Response(gen_image_stream('mask'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/viz_stream')
def viz_stream():
    return Response(gen_image_stream('viz'),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    t = threading.Thread(target=camera_loop, daemon=True)
    t.start()
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

#sudo /home/pi8/wrofe2025/env_test/bin/python /home/pi8/wrofe2025/main5.py