# obs_main_test2.py

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
    from image_frame_test3 import process_frame_for_steering
    from robot_motion import adjust_servo_angle, robot_forward_speed, robot_stop, robot_forward, motor_standby
    from lidar_steering2 import LidarScanner, PIDController, calculate_steering_error
except ImportError as e:
    print(f"Error importing modules: {e}")
    sys.exit()

# --- Global Variables ---
output_frame = None
output_frame_lock = threading.Lock() # Lock for thread-safe access to output_frame for Flask streaming

# Shared LiDAR data buffer and its lock
latest_lidar_data = {}
lidar_data_lock = threading.Lock()

app = Flask(__name__)

# --- CONTROL CONSTANTS ---
SERVO_CENTER_ANGLE = 90
STEERING_GAIN = 0.1 # For camera-based steering (adjust if robot over/under steers)
ROBOT_SPEED = 0.7  # Base speed for the robot (adjust as needed for desired pace)

# --- CAMERA CONFIGURATION ---
CAMERA_RESOLUTION = (768, 432) # Keeping this as it's efficient for your system
CAMERA_FRAMERATE = 30.0       # Target camera framerate
CAMERA_BUFFER_COUNT = 2       # Number of buffers for camera frames

# --- LIDAR CONTROL CONSTANTS ---
LIDAR_TARGET_DISTANCE_MM = 750 # Desired distance from the wall in millimeters
LIDAR_SAFETY_DISTANCE_MM = 300 # If any point in front is closer than this, command emergency stop
CLOCKWISE_WALL_FOLLOWING = True # True for right wall, False for left wall

# PID parameters for LiDAR (THESE WILL LIKELY NEED FINE-TUNING!)
if CLOCKWISE_WALL_FOLLOWING:
    LIDAR_PID_KP = 0.3
    LIDAR_PID_KI = 0.001
    LIDAR_PID_KD = 0.02
else: # Anticlockwise (left wall following)
    LIDAR_PID_KP = 0.2
    LIDAR_PID_KI = 0.001
    LIDAR_PID_KD = 0.05

# Servo mapping for LiDAR (from test_main_copy - adjust min/max if needed for your servo)
LIDAR_SERVO_MIN_ANGLE = 70
LIDAR_SERVO_MAX_ANGLE = 120
LIDAR_STEERING_SCALE_FACTOR = 0.4 # Adjust this to control steering sensitivity from PID output

# --- DEBUGGING AND UI ---
DEBUG_UI_OVERLAYS = True
PROFILE_PERFORMANCE = True # Set to False to disable periodic console profiling reports

# --- BEHAVIOR STATES ---
class RobotState:
    RED_AVOIDANCE = "RED_AVOIDANCE" # NEW: Highest priority visual state
    GREEN_AVOIDANCE = "GREEN_AVOIDANCE"
    LIDAR_WALL_FOLLOWING = "LIDAR_WALL_FOLLOWING"
    STOP = "STOP"
    INITIALIZING = "INITIALIZING"
    FALLBACK_STRAIGHT = "FALLBACK_STRAIGHT"

current_robot_state = RobotState.INITIALIZING # Initial state of the robot

# Helper function to map PID output to servo angle for LiDAR
def map_lidar_steering_angle(center_angle, pid_output, clockwise=True):
    adjusted_output = pid_output * LIDAR_STEERING_SCALE_FACTOR
    if clockwise:
        angle = center_angle - adjusted_output
    else:
        angle = center_angle + adjusted_output
    return max(LIDAR_SERVO_MIN_ANGLE, min(angle, LIDAR_SERVO_MAX_ANGLE))

# --- LiDAR Data Acquisition Thread Function ---
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
    global output_frame, output_frame_lock, current_robot_state

    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(
        main={"size": CAMERA_RESOLUTION},
        transform=libcamera.Transform(vflip=True, hflip=True),
        controls={"FrameRate": CAMERA_FRAMERATE},
        buffer_count=CAMERA_BUFFER_COUNT
    )
    picam2.configure(camera_config)
    picam2.start()
    print(f"Camera started with resolution {CAMERA_RESOLUTION} at {CAMERA_FRAMERATE} FPS.")

    lidar_scanner, lidar_pid, lidar_acquisition_thread = None, None, None
    try:
        lidar_scanner = LidarScanner()
        lidar_scanner.connect()
        lidar_acquisition_thread = threading.Thread(target=lidar_acquisition_thread_func, args=(lidar_scanner,))
        lidar_acquisition_thread.daemon = True
        lidar_acquisition_thread.start()
        lidar_pid = PIDController(Kp=LIDAR_PID_KP, Ki=LIDAR_PID_KI, Kd=LIDAR_PID_KD, setpoint=0)
        print("LiDAR system initialized successfully.")
    except IOError as e:
        print(f"WARNING: Failed to initialize LiDAR system: {e}.")
        lidar_scanner = None

    current_robot_state = RobotState.LIDAR_WALL_FOLLOWING if lidar_scanner else RobotState.FALLBACK_STRAIGHT
    print(f"Initial Robot State: {current_robot_state}")

    # Performance Profiling Variables
    frame_counter, total_loop_time, total_capture_time, total_process_time, total_lidar_read_time, total_control_time = 0, 0, 0, 0, 0, 0
    report_interval = 5.0
    last_report_time = time.monotonic()

    try:
        while True:
            loop_start_time = time.monotonic()

            # 1. Capture Camera Frame
            capture_start = time.monotonic()
            frame_bgr = cv2.cvtColor(picam2.capture_array(), cv2.COLOR_RGB2BGR)
            total_capture_time += (time.monotonic() - capture_start)

            # 2. Process Camera Frame for Obstacle Detection
            process_start = time.monotonic()
            # The function now returns 5 items, which matches the original call signature
            processed_frame, vision_angle, _, logic_label, _ = process_frame_for_steering(frame_bgr)
            total_process_time += (time.monotonic() - process_start)

            if processed_frame is None:
                print("Warning: process_frame_for_steering returned None.")
                with output_frame_lock:
                    output_frame = frame_bgr.copy()
                time.sleep(0.001)
                continue

            target_servo_angle = SERVO_CENTER_ANGLE
            robot_speed_current = ROBOT_SPEED
            display_text = ""

            # --- 3. BEHAVIOR ARBITRATION LOGIC ---
            control_start = time.monotonic()

            if logic_label == "red_obstacle":
                # Behavior 0: Red obstacle detected (HIGHEST PRIORITY)
                current_robot_state = RobotState.RED_AVOIDANCE
                servo_adjust = -vision_angle * STEERING_GAIN
                target_servo_angle = SERVO_CENTER_ANGLE - servo_adjust
                # Round off the servo angle as requested
                target_servo_angle = int(round(np.clip(target_servo_angle, 60, 120)))
                robot_speed_current = ROBOT_SPEED
                display_text = f"MODE: RedAvoid | Steer: {target_servo_angle}°"

            elif logic_label == "obstacle":
                # Behavior 1: Green obstacle detected
                current_robot_state = RobotState.GREEN_AVOIDANCE
                servo_adjust = -vision_angle * STEERING_GAIN
                target_servo_angle = SERVO_CENTER_ANGLE - servo_adjust
                # Round off the servo angle as requested
                target_servo_angle = int(round(np.clip(target_servo_angle, 60, 120)))
                robot_speed_current = ROBOT_SPEED
                display_text = f"MODE: CamAvoid | Steer: {target_servo_angle}°"
            
            elif lidar_scanner and lidar_pid:
                # Behavior 2: LiDAR Wall Following
                current_robot_state = RobotState.LIDAR_WALL_FOLLOWING
                lidar_read_start = time.monotonic()
                with lidar_data_lock:
                    scan_data = latest_lidar_data.copy()
                total_lidar_read_time += (time.monotonic() - lidar_read_start)

                if scan_data:
                    lidar_error = calculate_steering_error(scan_data, LIDAR_TARGET_DISTANCE_MM, LIDAR_SAFETY_DISTANCE_MM)
                    if lidar_error == 9999.0:
                        if current_robot_state != RobotState.STOP:
                            print("LiDAR: EMERGENCY! Front obstacle detected. Commanding STOP.")
                            robot_stop()
                        current_robot_state = RobotState.STOP
                        display_text = "MODE: STOP (LiDAR Obstacle!)"
                        time.sleep(0.1)
                        continue
                    else:
                        pid_output = lidar_pid.update(lidar_error)
                        target_servo_angle = map_lidar_steering_angle(SERVO_CENTER_ANGLE, pid_output, CLOCKWISE_WALL_FOLLOWING)
                        robot_speed_current = ROBOT_SPEED
                        display_text = f"MODE: LiDARWF | Steer: {round(target_servo_angle)}° | Err: {lidar_error:.0f}mm"
                else:
                    # LiDAR initialized but no data yet
                    current_robot_state = RobotState.FALLBACK_STRAIGHT
                    target_servo_angle = SERVO_CENTER_ANGLE
                    display_text = "MODE: Fallback (No LiDAR Data)"
            else:
                # Behavior 3: Fallback (No green/red obstacle, No LiDAR)
                current_robot_state = RobotState.FALLBACK_STRAIGHT
                target_servo_angle = SERVO_CENTER_ANGLE
                robot_speed_current = ROBOT_SPEED
                display_text = f"MODE: Fallback | Logic: {logic_label}"

            total_control_time += (time.monotonic() - control_start)

            # 4. Apply Robot Motion
            if current_robot_state != RobotState.STOP:
                adjust_servo_angle(target_servo_angle)
                robot_forward_speed(robot_speed_current)
            else:
                robot_stop()

            # 5. UI Overlays
            if DEBUG_UI_OVERLAYS:
                loop_duration = time.monotonic() - loop_start_time
                fps = 1.0 / loop_duration if loop_duration > 0 else 0
                cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(processed_frame, f"State: {current_robot_state}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(processed_frame, f"FPS: {fps:.1f}", (processed_frame.shape[1] - 120, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            
            # 6. Update Output Frame for Flask
            with output_frame_lock:
                output_frame = processed_frame.copy()

            # 7. Performance Profiling
            if PROFILE_PERFORMANCE:
                frame_counter += 1
                total_loop_time += (time.monotonic() - loop_start_time)
                if (time.monotonic() - last_report_time) >= report_interval:
                    # Print performance report (code omitted for brevity but is unchanged)
                    # ...
                    # Reset counters
                    frame_counter, total_loop_time, total_capture_time, total_process_time, total_lidar_read_time, total_control_time = 0, 0, 0, 0, 0, 0
                    last_report_time = time.monotonic()

            time.sleep(0.001)

    finally:
        print("Control loop ending...")
        robot_stop()
        picam2.stop()
        print("Robot, camera, and LiDAR resources released.")


# --- Flask Streaming Functions (Unchanged) ---
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

# --- Main Execution Block ---
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