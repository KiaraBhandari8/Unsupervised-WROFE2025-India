import cv2
import sys
import numpy as np
from picamera2 import Picamera2
import libcamera
from flask import Flask, render_template, Response
import threading
import time
import os

# Import your custom functions
try:
    from image_frames_asp import analyze_scene
    from robot_motion import adjust_servo_angle, robot_forward_speed, robot_stop, motor_standby
    from lidar_steering2 import LidarScanner, PIDController, calculate_steering_error
except ImportError as e:
    print(f"Error importing modules: {e}")
    sys.exit()

# --- Global Variables ---
output_frame = None
output_frame_lock = threading.Lock()
latest_lidar_data = {}
lidar_data_lock = threading.Lock()

# --- FLASK WEB SERVER SETUP ---
app = Flask(__name__)

# --- ROBOT MISSION CONSTANTS ---
TOTAL_LAPS = 3
CORNERS_PER_LAP = 4
TOTAL_TURNS_TO_COMPLETE = TOTAL_LAPS * CORNERS_PER_LAP

# --- MOTION & STEERING CONSTANTS ---
ROBOT_SPEED = 0.75
STRAIGHT_ANGLE = 90
SHARP_TURN_DEVIATION = 40
SHARP_RIGHT = STRAIGHT_ANGLE - SHARP_TURN_DEVIATION
SHARP_LEFT = STRAIGHT_ANGLE + SHARP_TURN_DEVIATION
SERVO_MIN_ANGLE = 60
SERVO_MAX_ANGLE = 120

# --- VISION-BASED CONTROL GAINS ---
RED_TARGET_X = 110
GREEN_TARGET_X = 660 
VISION_KP = 0.5
VISION_KD = 0.1
VISION_CY = 0.08
WALL_KP = 0.4
WALL_KD = 0.1

# --- LIDAR CONTROL CONSTANTS ---
LIDAR_ENABLED = True
LIDAR_TARGET_DISTANCE_MM = 750
LIDAR_SAFETY_DISTANCE_MM = 300
LIDAR_PID_KP = 0.3
LIDAR_PID_KI = 0.001
LIDAR_PID_KD = 0.02
LIDAR_STEERING_SCALE_FACTOR = 0.2

# --- CAMERA CONFIGURATION ---
CAMERA_RESOLUTION = (768, 432) # (width, height)
CAMERA_FRAMERATE = 30.0
DEBUG_UI_OVERLAYS = True


# --- REGIONS OF INTEREST (ROIs) ---
FRAME_W, FRAME_H = CAMERA_RESOLUTION
ROIS = {
    'wall_left': (0, int(FRAME_H * 0.45), FRAME_W // 2, int(FRAME_H * 0.8)),
    'wall_right': (FRAME_W // 2, int(FRAME_H * 0.45), FRAME_W, int(FRAME_H * 0.8)),
    'pillar': (int(FRAME_W * 0.1), int(FRAME_H * 0.25), int(FRAME_W * 0.9), int(FRAME_H * 0.9)),
    'line': (int(FRAME_W * 0.25), int(FRAME_H * 0.6), int(FRAME_W * 0.75), FRAME_H)
}


# Helper function to map PID output to servo angle for LiDAR
def map_lidar_steering_angle(center_angle, pid_output):
    adjusted_output = pid_output * LIDAR_STEERING_SCALE_FACTOR
    angle = center_angle - adjusted_output
    return max(SERVO_MIN_ANGLE, min(angle, SERVO_MAX_ANGLE))

# LiDAR Data Acquisition Thread Function
def lidar_acquisition_thread_func(scanner_instance):
    global latest_lidar_data, lidar_data_lock
    print("LiDAR acquisition thread started.")
    try:
        while True:
            data = scanner_instance.get_scan_data()
            if data:
                with lidar_data_lock:
                    latest_lidar_data = data
            time.sleep(0.01)
    except Exception as e:
        print(f"LiDAR Acquisition Thread Error: {e}")
    finally:
        print("LiDAR acquisition thread stopping.")
        if scanner_instance:
            scanner_instance.disconnect()

def robot_control_loop():
    global output_frame, output_frame_lock

    # Initialization
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(main={"size": CAMERA_RESOLUTION}, transform=libcamera.Transform(vflip=True, hflip=True), controls={"FrameRate": CAMERA_FRAMERATE})
    picam2.configure(camera_config)
    picam2.start()
    print(f"Camera started with resolution {CAMERA_RESOLUTION}.")

    lidar_scanner, lidar_pid, lidar_acquisition_thread = None, None, None
    if LIDAR_ENABLED:
        try:
            lidar_scanner = LidarScanner()
            lidar_scanner.connect()
            lidar_acquisition_thread = threading.Thread(target=lidar_acquisition_thread_func, args=(lidar_scanner,))
            lidar_acquisition_thread.daemon = True
            lidar_acquisition_thread.start()
            lidar_pid = PIDController(Kp=LIDAR_PID_KP, Ki=LIDAR_PID_KI, Kd=LIDAR_PID_KD, setpoint=0)
            print("LiDAR system initialized successfully.")
        except IOError as e:
            print(f"WARNING: Failed to initialize LiDAR: {e}. Running in vision-only mode.")
            lidar_scanner = None

    # Mission State Variables
    t = 0
    turnDir = "none"
    lTurn, rTurn = False, False

    # PD Controller State Variables
    vision_prev_error = 0.0
    wall_prev_diff = 0.0
    
    current_mode = "Initializing"

    try:
        while True:
            frame_bgr = cv2.cvtColor(picam2.capture_array(), cv2.COLOR_RGB2BGR)
            
            processed_frame, scene_data = analyze_scene(frame_bgr, ROIS)

            if not scene_data:
                time.sleep(0.01)
                continue

            # Mission Completion Check
            if t >= TOTAL_TURNS_TO_COMPLETE:
                if current_mode != "STOPPED":
                    print(f"Mission Complete! {TOTAL_LAPS} Laps finished. Stopping.")
                    robot_stop()
                    current_mode = "STOPPED"
                
                if DEBUG_UI_OVERLAYS:
                    cv2.putText(processed_frame, "MISSION COMPLETE", (120, 220), cv2.FONT_HERSHEY_TRIPLEX, 2, (0, 255, 0), 4)
                with output_frame_lock:
                    output_frame = processed_frame.copy()
                time.sleep(0.1)
                continue
            
            pillar = scene_data["closest_pillar"]
            angle = STRAIGHT_ANGLE

            # --- BEHAVIOR ARBITRATION LOGIC ---
            if lTurn or rTurn:
                current_mode = f"Cornering {'Left' if lTurn else 'Right'}"
                exit_thresh = 4000
                if (rTurn and scene_data["left_wall_area"] >= exit_thresh) or \
                   (lTurn and scene_data["right_wall_area"] >= exit_thresh):
                    lTurn, rTurn = False, False
                    t += 1
                    print(f"--- Turn {t} Complete ---")
                else:
                    angle = SHARP_LEFT if lTurn else SHARP_RIGHT
            elif pillar.color is not None:
                if pillar.color == 'red':
                    target_x = RED_TARGET_X
                    current_mode = "Pillar: Red -> Steer RIGHT"
                else: 
                    target_x = GREEN_TARGET_X
                    current_mode = "Pillar: Green -> Steer LEFT"

                vision_error = target_x - pillar.x
                angle = STRAIGHT_ANGLE + (vision_error * VISION_KP) + ((vision_error - vision_prev_error) * VISION_KD)
                ROI_TOP_Y = ROIS['pillar'][1]
                y_correction = VISION_CY * (pillar.y - ROI_TOP_Y)
                angle += -y_correction if vision_error <= 0 else y_correction
                vision_prev_error = vision_error
                wall_prev_diff = 0
            else:
                vision_prev_error = 0.0
                if turnDir == "none":
                    if scene_data["orange_line_area"] > 100: turnDir = "right"
                    elif scene_data["blue_line_area"] > 100: turnDir = "left"
                
                if (turnDir == "right" and scene_data["orange_line_area"] > 100) or \
                   (turnDir == "left" and scene_data["blue_line_area"] > 100):
                    lTurn = (turnDir == "left")
                    rTurn = (turnDir == "right")
                    angle = SHARP_LEFT if lTurn else SHARP_RIGHT
                elif lidar_scanner:
                    current_mode = "LiDAR Wall Follow"
                    with lidar_data_lock: scan_data = latest_lidar_data.copy()
                    if scan_data:
                        lidar_error = calculate_steering_error(scan_data, LIDAR_TARGET_DISTANCE_MM, LIDAR_SAFETY_DISTANCE_MM)
                        if lidar_error == 9999.0:
                             robot_stop()
                             current_mode = "STOP (LiDAR Obstacle)"
                             time.sleep(0.1)
                             continue
                        else:
                            pid_output = lidar_pid.update(lidar_error)
                            angle = map_lidar_steering_angle(STRAIGHT_ANGLE, pid_output)
                else:
                    current_mode = "Lane Keeping (Vision)"
                    wall_diff = scene_data["right_wall_area"] - scene_data["left_wall_area"]
                    angle = STRAIGHT_ANGLE + (wall_diff * WALL_KP) + ((wall_diff - wall_prev_diff) * WALL_KD)
                    wall_prev_diff = wall_diff

            # --- Final Actuation ---
            final_angle = np.clip(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)
            adjust_servo_angle(final_angle)
            robot_forward_speed(ROBOT_SPEED)

            # Update UI Overlays
            if DEBUG_UI_OVERLAYS:
                cv2.putText(processed_frame, f"Mode: {current_mode}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(processed_frame, f"Turns: {t}/{TOTAL_TURNS_TO_COMPLETE}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                cv2.putText(processed_frame, f"Angle: {int(final_angle)}", (processed_frame.shape[1] - 150, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            with output_frame_lock:
                output_frame = processed_frame.copy()

    finally:
        print("Control loop ending...")
        robot_stop()
        picam2.stop()
        print("Robot and camera resources released.")

# --- Flask Streaming Functions and Main Execution Block ---
def generate_frames():
    global output_frame, output_frame_lock
    while True:
        with output_frame_lock:
            if output_frame is None:
                time.sleep(0.01)
                continue
            (flag, encoded_image) = cv2.imencode(".jpg", output_frame)
            if not flag:
                continue
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encoded_image) + b'\r\n')

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == '__main__':
    print("--- Starting Robot Control System ---")
    robot_stop()
    motor_standby()
    time.sleep(0.5)

    control_thread = threading.Thread(target=robot_control_loop)
    control_thread.daemon = True
    control_thread.start()
    print("Robot control thread started.")

    hostname = os.uname()[1]
    print(f"Web server starting. Open http://{hostname}.local:5000 or http://<raspberrypi_ip>:5000")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)

    print("Main application exiting.")