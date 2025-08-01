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

# --- DIRECTION FLAG ---
# Set this to True for original direction, False for opposite direction
facing_original_direction = False  # CHANGE THIS as needed

def camera_loop():
    global latest_raw_jpeg, latest_viz_jpeg, latest_angle, latest_final_steering_angle, latest_command
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(main={"size": (768, 432)},
                                                            transform=libcamera.Transform(vflip=True, hflip=True))
    picam2.configure(camera_config)
    picam2.start()
    time.sleep(2)  # Camera warm-up

    while True:
        frame = picam2.capture_array()
        command, steering_angle, viz_frame = get_robot_direction_and_angle(frame)
        if steering_angle is None:
            robot_motion.robot_stop()
        else:
            steering_angle = int(steering_angle)

            # --- INVERT STEERING ANGLE IF FACING OPPOSITE DIRECTION ---
            if not facing_original_direction:
                steering_angle = -steering_angle

            # if steering_angle > 0:
            #     steering_angle = steering_angle * 2
            print(f"Weighted Steering Angle : {int(steering_angle)}")
            final_steering_angle = int(75 + (-1 * int(steering_angle)))
            print(f"Final Steering Angle : {int(final_steering_angle)}")

        # Call robot drive functions
        if command in ("FORWARD", "LEFT", "RIGHT"):
            robot_motion.robot_forward()
            robot_motion.adjust_servo_angle(final_steering_angle)
        else:
            robot_motion.robot_stop()
         
        # Encode images for streaming
        _, raw_jpeg = cv2.imencode('.jpg', frame)
        _, viz_jpeg = cv2.imencode('.jpg', viz_frame if viz_frame is not None else frame)

        with lock:
            latest_raw_jpeg = raw_jpeg.tobytes()
            latest_viz_jpeg = viz_jpeg.tobytes()
            latest_angle = steering_angle
            latest_final_steering_angle = final_steering_angle
            latest_command = command

        time.sleep(0.1)  # Adjust as needed

# ... (rest of your Flask code remains unchanged)

