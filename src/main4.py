import io
import time
import threading
import cv2
import numpy as np
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2
import robot_motion
from process_frames import get_robot_direction_and_angle
import libcamera

app = Flask(__name__)

# --- Camera Initialization (GLOBAL, SINGLE INSTANCE) ---
picam2 = Picamera2()
camera_config = picam2.create_preview_configuration(
    main={"size": (640, 480)},
    transform=libcamera.Transform(vflip=True, hflip=True)
)
picam2.configure(camera_config)
picam2.start()
time.sleep(2)  # Camera warm-up

# --- Shared Data ---
latest_raw_jpeg = None
latest_viz_jpeg = None
latest_angle = 0
latest_final_steering_angle = 75
latest_command = "STOP"
lock = threading.Lock()
facing_original_direction = True

def get_blank_jpeg(width=640, height=480):
    blank = (255 * np.ones((height, width, 3), dtype=np.uint8))
    _, jpeg = cv2.imencode('.jpg', blank)
    return jpeg.tobytes()

def camera_loop():
    global latest_raw_jpeg, latest_viz_jpeg, latest_angle, latest_final_steering_angle, latest_command
    while True:
        try:
            frame = picam2.capture_array()
            command, steering_angle, viz_frame = get_robot_direction_and_angle(frame)
            if steering_angle is None:
                robot_motion.robot_stop()
                final_steering_angle = 75
                steering_angle = 0
            else:
                steering_angle = int(steering_angle)
                if not facing_original_direction:
                    steering_angle = -steering_angle
                if steering_angle > 0:
                    steering_angle *= 3
                final_steering_angle = int(75 + (-steering_angle))
            if command in ("FORWARD", "LEFT", "RIGHT"):
                robot_motion.robot_forward()
                robot_motion.adjust_servo_angle(final_steering_angle)
            else:
                robot_motion.robot_stop()
            _, raw_jpeg = cv2.imencode('.jpg', frame)
            _, viz_jpeg = cv2.imencode('.jpg', viz_frame if viz_frame is not None else frame)
            with lock:
                latest_raw_jpeg = raw_jpeg.tobytes()
                latest_viz_jpeg = viz_jpeg.tobytes()
                latest_angle = steering_angle
                latest_final_steering_angle = final_steering_angle
                latest_command = command
        except Exception as e:
            print(f"Camera loop error: {e}")
        time.sleep(0.1)

HTML_PAGE = """..."""  # Your HTML remains unchanged

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
        if frame is not None:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + get_blank_jpeg() + b'\r\n')
        time.sleep(0.1)

@app.route('/raw_stream')
def raw_stream():
    return Response(gen_image_stream('raw'), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/viz_stream')
def viz_stream():
    return Response(gen_image_stream('viz'), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    with lock:
        latest_raw_jpeg = get_blank_jpeg()
        latest_viz_jpeg = get_blank_jpeg()
    t = threading.Thread(target=camera_loop, daemon=True)
    t.start()
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
