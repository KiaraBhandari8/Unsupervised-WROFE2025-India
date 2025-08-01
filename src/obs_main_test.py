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
    from image_frame_test import process_frame_for_steering
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
ROBOT_SPEED = 0.8  # Base speed for the robot (adjust as needed for desired pace)

# --- CAMERA CONFIGURATION ---
CAMERA_RESOLUTION = (640, 360) # Keeping this as it's efficient for your system
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
LIDAR_STEERING_SCALE_FACTOR = 0.40 # Adjust this to control steering sensitivity from PID output

# --- DEBUGGING AND UI ---
DEBUG_UI_OVERLAYS = True
PROFILE_PERFORMANCE = True # Set to False to disable periodic console profiling reports

# --- BEHAVIOR STATES ---
class RobotState:
    CAMERA_AVOIDANCE = "CAMERA_AVOIDANCE"
    LIDAR_WALL_FOLLOWING = "LIDAR_WALL_FOLLOWING"
    STOP = "STOP"
    INITIALIZING = "INITIALIZING"
    FALLBACK_STRAIGHT = "FALLBACK_STRAIGHT" # No green obstacle, no LiDAR, or LiDAR data issues

current_robot_state = RobotState.INITIALIZING # Initial state of the robot

# Helper function to map PID output to servo angle for LiDAR
def map_lidar_steering_angle(center_angle, pid_output, clockwise=True):
    adjusted_output = pid_output * LIDAR_STEERING_SCALE_FACTOR

    if clockwise:
        angle = center_angle - adjusted_output
    else:
        angle = center_angle + adjusted_output

    return max(LIDAR_SERVO_MIN_ANGLE, min(angle, LIDAR_SERVO_MAX_ANGLE))

# --- NEW: Dedicated LiDAR Data Acquisition Thread Function ---
def lidar_acquisition_thread_func(scanner_instance):
    global latest_lidar_data, lidar_data_lock
    print("LiDAR acquisition thread started.")
    try:
        while True:
            data = scanner_instance.get_scan_data()
            if data:
                with lidar_data_lock:
                    latest_lidar_data = data
            # Small sleep to avoid busy-waiting, but allow LiDAR to update
            # The LiDAR itself has a scan frequency (e.g., 10Hz = 100ms per scan).
            # Sleeping less than that means you're just polling, not necessarily getting new data.
            # Sleeping slightly more than the scan frequency might be optimal to get fresh data.
            time.sleep(0.01) # Poll for new data every 10ms
    except Exception as e:
        print(f"LiDAR Acquisition Thread Error: {e}")
    finally:
        print("LiDAR acquisition thread stopping.")
        if scanner_instance:
            scanner_instance.disconnect()


def robot_control_loop():
    global output_frame, output_frame_lock, current_robot_state

    # --- Initialize Picamera2 ---
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

    # --- Initialize LiDAR and PID for LiDAR control (main thread, just for init) ---
    lidar_scanner = None
    lidar_pid = None
    lidar_acquisition_thread = None # Reference for the new thread
    try:
        lidar_scanner = LidarScanner()
        lidar_scanner.connect() # Attempt connection in main thread
        
        # Start the dedicated LiDAR acquisition thread
        lidar_acquisition_thread = threading.Thread(target=lidar_acquisition_thread_func, args=(lidar_scanner,))
        lidar_acquisition_thread.daemon = True # Allow main program to exit if this thread is running
        lidar_acquisition_thread.start()

        lidar_pid = PIDController(
            Kp=LIDAR_PID_KP,
            Ki=LIDAR_PID_KI,
            Kd=LIDAR_PID_KD,
            setpoint=0
        )
        print("LiDAR system (scanner + acquisition thread + PID) initialized successfully.")
    except IOError as e:
        print(f"WARNING: Failed to initialize LiDAR system: {e}. Robot will operate with camera and fallback behaviors only.")
        lidar_scanner = None # Ensure it's None if connection fails
        # No need to stop a thread if it wasn't started due to connection failure

    # Set initial state based on LiDAR availability
    current_robot_state = RobotState.LIDAR_WALL_FOLLOWING if lidar_scanner else RobotState.FALLBACK_STRAIGHT
    print(f"Initial Robot State: {current_robot_state}")

    # --- Performance Profiling Variables ---
    frame_counter = 0
    total_loop_time = 0
    total_capture_time = 0
    total_process_time = 0
    total_lidar_read_time = 0 # Time to read from shared buffer, not acquire
    total_control_time = 0
    report_interval = 5.0
    last_report_time = time.monotonic()

    try:
        while True:
            loop_start_time = time.monotonic()

            # --- 1. Capture Camera Frame ---
            capture_start = time.monotonic()
            frame_rgb = picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            capture_end = time.monotonic()
            total_capture_time += (capture_end - capture_start)

            # --- 2. Process Camera Frame for Obstacle Detection ---
            process_start = time.monotonic()
            processed_frame, vision_angle, _, logic_label, _ = process_frame_for_steering(frame_bgr)
            process_end = time.monotonic()
            total_process_time += (process_end - process_start)

            if processed_frame is None:
                print("Warning: process_frame_for_steering returned None. Using raw frame for display.")
                with output_frame_lock:
                    output_frame = frame_bgr.copy()
                time.sleep(0.001) # Minimal sleep if processing fails
                continue

            target_servo_angle = SERVO_CENTER_ANGLE
            robot_speed_current = ROBOT_SPEED
            display_text = ""

            # --- 3. BEHAVIOR ARBITRATION LOGIC ---
            control_start = time.monotonic()

            if logic_label == "obstacle":
                # Behavior 1: Camera detected a green obstacle (HIGH PRIORITY)
                current_robot_state = RobotState.CAMERA_AVOIDANCE
                servo_adjust = -vision_angle * STEERING_GAIN
                target_servo_angle = int(np.clip(SERVO_CENTER_ANGLE - servo_adjust, 60, 120))
                robot_speed_current = ROBOT_SPEED
                display_text = f"MODE: CamAvoid | Steer: {target_servo_angle}° | Logic: {logic_label}"

            elif lidar_scanner and lidar_pid:
                # Behavior 2: No green obstacle, and LiDAR system is initialized.
                current_robot_state = RobotState.LIDAR_WALL_FOLLOWING

                # Read latest LiDAR data from the shared buffer (fast operation)
                lidar_read_start = time.monotonic()
                with lidar_data_lock:
                    scan_data = latest_lidar_data.copy() # Get a copy of the latest data
                lidar_read_end = time.monotonic()
                total_lidar_read_time += (lidar_read_end - lidar_read_start)

                if scan_data:
                    lidar_error = calculate_steering_error(scan_data,
                                                           target_distance_mm=LIDAR_TARGET_DISTANCE_MM,
                                                           safety_distance_mm=LIDAR_SAFETY_DISTANCE_MM)

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
                        display_text = (f"MODE: LiDARWF | Steer: {target_servo_angle}° | "
                                        f"LiDAR Error: {lidar_error:.0f}mm")
                else:
                    # LiDAR thread is running, but no data has been acquired yet, or it's empty
                    print("LiDAR: No scan data available from thread. Falling back to straight.")
                    current_robot_state = RobotState.FALLBACK_STRAIGHT
                    target_servo_angle = SERVO_CENTER_ANGLE
                    robot_speed_current = ROBOT_SPEED  
                    display_text = "MODE: Fallback (No LiDAR Data Yet)"
            else:
                # Behavior 3: No green obstacle, AND LiDAR system failed to initialize
                current_robot_state = RobotState.FALLBACK_STRAIGHT
                target_servo_angle = SERVO_CENTER_ANGLE
                robot_speed_current = ROBOT_SPEED * 1
                display_text = "MODE: Fallback (No LiDAR Hardware)"

            control_end = time.monotonic()
            total_control_time += (control_end - control_start)


            # --- 4. Apply Robot Motion based on current_robot_state ---
            if current_robot_state != RobotState.STOP:
                adjust_servo_angle(target_servo_angle)
                robot_forward_speed(robot_speed_current)
            else:
                robot_stop()

            # --- 5. UI Overlays (Conditional for Performance) ---
            if DEBUG_UI_OVERLAYS:
                cv2.putText(processed_frame, display_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                cv2.putText(processed_frame, f"State: {current_robot_state}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
                loop_end_time = time.monotonic()
                loop_duration = loop_end_time - loop_start_time
                fps = 1.0 / loop_duration if loop_duration > 0 else 0
                cv2.putText(processed_frame, f"FPS: {fps:.1f}", (processed_frame.shape[1] - 120, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            # --- 6. Update Output Frame for Flask Stream ---
            with output_frame_lock:
                output_frame = processed_frame.copy()

            # --- 7. Performance Profiling Report (Conditional) ---
            if PROFILE_PERFORMANCE:
                frame_counter += 1
                total_loop_time += (time.monotonic() - loop_start_time)
                if (time.monotonic() - last_report_time) >= report_interval:
                    avg_loop_time = total_loop_time / frame_counter
                    avg_capture_time = total_capture_time / frame_counter
                    avg_process_time = total_process_time / frame_counter
                    avg_lidar_read_time = total_lidar_read_time / frame_counter
                    avg_control_time = total_control_time / frame_counter

                    print(f"\n--- Performance Report (Avg over {report_interval}s) ---")
                    print(f"Total Frames Processed: {frame_counter}")
                    print(f"Avg Loop FPS: {1.0 / avg_loop_time:.2f} (Avg Loop Time: {avg_loop_time*1000:.2f}ms)")
                    print(f"  Avg Capture (Cam):   {avg_capture_time*1000:.2f}ms")
                    print(f"  Avg Process (CV):    {avg_process_time*1000:.2f}ms")
                    print(f"  Avg LiDAR (Read):    {avg_lidar_read_time*1000:.2f}ms") # Renamed for clarity
                    print(f"  Avg Control Logic:   {avg_control_time*1000:.2f}ms")
                    print("-----------------------------------\n")

                    # Reset counters for the next interval
                    frame_counter = 0
                    total_loop_time = 0
                    total_capture_time = 0
                    total_process_time = 0
                    total_lidar_read_time = 0
                    total_control_time = 0
                    last_report_time = time.monotonic()

            # --- 8. Minimal Sleep to Yield CPU ---
            time.sleep(0.001) # Yield to other threads (Flask, LiDAR acquisition)

    finally:
        print("Control loop ending. Stopping robot and shutting down sensors.")
        robot_stop()
        # The LiDAR acquisition thread will automatically disconnect its scanner
        # when its `while True` loop is broken (daemon thread).
        picam2.stop()
        print("Robot, camera, and LiDAR resources released.")

# --- Flask Streaming Functions (Unchanged) ---
def generate_frames():
    global output_frame, output_frame_lock # Use the correct lock
    while True:
        with output_frame_lock: # Acquire lock before accessing output_frame
            if output_frame is None:
                time.sleep(0.01)
                continue

            (flag, encoded_image) = cv2.imencode(".jpg", output_frame)
            if not flag:
                print("Failed to encode frame for streaming.")
                continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' +
               bytearray(encoded_image) + b'\r\n')

@app.route("/")
def index():
    # You'll need an 'index.html' file in a 'templates' folder next to this script.
    return render_template("index.html")

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

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

    print(f"Web server starting. Open http://{os.uname()[1]}.local:5000 or http://<raspberrypi_ip>:5000")
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)

    print("Main application exiting.")