import time
import cv2
import numpy as np
import robot_motion_sample
from picamera2 import Picamera2
from libcamera import Transform
from lidar_steering1 import LidarScanner, PIDController, calculate_steering_error

# =============================================================================
# --- Camera and Color Detection Functions (from line_tracking.py) ---
# =============================================================================

def filter_blue_objects(hsv_frame):
    """
    Processes an HSV image to find and count distinct blue objects.
    """
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([130, 255, 255])
    
    blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
    
    kernel = np.ones((5, 5), np.uint8)
    blue_mask_eroded = cv2.erode(blue_mask, kernel, iterations=2)
    blue_mask_dilated = cv2.dilate(blue_mask_eroded, kernel, iterations=2)
    
    blue_contours, _ = cv2.findContours(blue_mask_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    return len(blue_contours)

# =============================================================================
# --- Steering and Navigation Functions (from test_main.py) ---
# =============================================================================

def map_steering_angle(center_angle, pid_output, clockwise=True):
    """Maps PID output to a servo angle, clamping it within safe limits."""
    scale_factor = 1.1 if clockwise else 1.0
    adjusted_output = pid_output * scale_factor

    if clockwise:
        angle = center_angle - adjusted_output
    else:
        angle = center_angle + adjusted_output

    min_angle = 50.0
    max_angle = 100.0
    return max(min_angle, min(angle, max_angle))

# =============================================================================
# --- Main Application ---
# =============================================================================

def main():
    print("=== Robot PID Navigation with Line Detection Initialized ===")
    
    # --- Configuration ---
    clockwise_mode = False
    center_angle = 75.0
    robot_motion_sample.set_motor_speed(0.5)
    
    # --- PID Setup ---
    if clockwise_mode:
        pid = PIDController(Kp=0.3, Ki=0.001, Kd=0.02)
    else:
        pid = PIDController(Kp=0.2, Ki=0.001, Kd=0.02)

    # --- Line Detection State ---
    line_counter = 0
    previous_blue_state = False
    
    # --- Hardware Initialization ---
    picam2 = Picamera2()
    config = picam2.create_still_configuration(
        main={"size": (640, 480)}, # Use a smaller resolution for faster processing
        transform=Transform(hflip=1, vflip=1)
    )
    picam2.configure(config)
    picam2.start()
    print("Camera started.")

    try:
        with LidarScanner() as scanner:
            print("LiDAR is active.")
            print(f"Direction: {'Clockwise' if clockwise_mode else 'Anticlockwise'}")

            while True:
                # --- Stop condition ---
                if line_counter >= 12:
                    print(f"Goal reached ({line_counter} line(s) passed). Stopping robot.")
                    break # Exit the main loop

                # 1. LiDAR-based Steering
                scan_data = scanner.get_scan_data()
                if scan_data:
                    error = calculate_steering_error(scan_data)
                    direction_error = -error if clockwise_mode else error
                    pid_output = pid.update(direction_error)
                    steering_angle = map_steering_angle(center_angle, pid_output, clockwise=clockwise_mode)
                    
                    robot_motion_sample.robot_forward()
                    robot_motion_sample.adjust_servo_angle(steering_angle)
                    
                    print(f"PID: {pid_output:.2f}, Servo Angle: {steering_angle:.2f}", end=' | ')
                else:
                    print("No LiDAR data. Stopping for safety.")
                    robot_motion_sample.robot_stop()
                    time.sleep(0.5) # Wait before retrying
                    continue

                # 2. Camera-based Line Detection
                frame = picam2.capture_array()
                hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
                blue_count = filter_blue_objects(hsv)
                
                current_blue_state = (blue_count > 0)
                
                # --- Counter Logic ---
                if not current_blue_state and previous_blue_state:
                    line_counter += 1
                    print(f"** Blue line passed! Counter: {line_counter} **")
                else:
                    print(f"Blue Detected: {current_blue_state}")

                previous_blue_state = current_blue_state
                
                time.sleep(0.1)

    except IOError as e:
        print(f"ERROR: Hardware issue: {e}")
    except KeyboardInterrupt:
        print("\nUser interrupted.")
    finally:
        print("Shutdown initiated...")
        robot_motion_sample.robot_stop()
        robot_motion_sample.motor_standby()
        picam2.stop()
        print("Camera stopped.")
        print("Shutdown complete.")

if __name__ == "__main__":
    main()
