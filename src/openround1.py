import time
import cv2
import numpy as np
from picamera2 import Picamera2
from lidar_steering2 import LidarScanner, PIDController, calculate_steering_error
import robot_motion

# --- Color Filtering Functions ---
def filter_blue_objects(hsv_frame):
    """Detects presence of blue using HSV masking."""
    lower_blue = np.array([80, 110, 50])
    upper_blue = np.array([130, 255, 255])
    blue_mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
    kernel = np.ones((5, 5), np.uint8)
    blue_mask = cv2.erode(blue_mask, kernel, iterations=2)
    blue_mask = cv2.dilate(blue_mask, kernel, iterations=2)
    return blue_mask

def filter_orange_objects(hsv_frame):
    """Detects presence of orange using HSV masking."""
    lower_orange = np.array([5, 100, 20])
    upper_orange = np.array([15, 255, 255])
    orange_mask = cv2.inRange(hsv_frame, lower_orange, upper_orange)
    kernel = np.ones((5, 5), np.uint8)
    orange_mask = cv2.erode(orange_mask, kernel, iterations=2)
    orange_mask = cv2.dilate(orange_mask, kernel, iterations=2)
    return orange_mask

def detect_color_binary(mask, threshold=1500):
    """Returns True if color is present above a pixel threshold."""
    return cv2.countNonZero(mask) > threshold

def map_steering_angle(center_angle, pid_output, clockwise=True):
    scale_factor = 0.40
    adjusted_output = pid_output * scale_factor
    angle = center_angle - adjusted_output if clockwise else center_angle + adjusted_output
    return int(max(70, min(angle, 110)))

def main():
    print("=== Robot PID Navigation & Color Line Detection ===")
    clockwise_mode = True
    center_angle = 90
    target_distance_mm = 750
    safety_distance_mm = 300
    max_color_count = 12

    # PID selection
    pid = PIDController(Kp=0.3, Ki=0.001, Kd=0.02, setpoint=0) if clockwise_mode else PIDController(Kp=0.2, Ki=0.001, Kd=0.05, setpoint=0)

    # Camera setup
    picam2 = Picamera2()
    config = picam2.create_video_configuration(main={"size": (768, 432)})
    picam2.configure(config)
    picam2.start()

    # Line crossing counters and states
    blue_count = 0
    orange_count = 0
    prev_blue_state = False
    prev_orange_state = False

    try:
        with LidarScanner() as scanner:
            print("LiDAR and camera active, starting navigation.")
            while True:
                # --- Get LiDAR scan data ---
                scan_data = scanner.get_scan_data()

                # --- Get camera frame ---
                frame = picam2.capture_array()
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

                # --- Color Detection ---
                blue_mask = filter_blue_objects(hsv)
                orange_mask = filter_orange_objects(hsv)
                blue_in_view = detect_color_binary(blue_mask)
                orange_in_view = detect_color_binary(orange_mask)

                # --- Blue Line Binary Logic ---
                if not blue_in_view and prev_blue_state:
                    blue_count += 1
                    print(f"Blue line crossed! Total blue lines: {blue_count}")
                prev_blue_state = blue_in_view

                # --- Orange Line Binary Logic ---
                if not orange_in_view and prev_orange_state:
                    orange_count += 1
                    print(f"Orange line crossed! Total orange lines: {orange_count}")
                prev_orange_state = orange_in_view

                # --- Stop robot if target line count reached (either color) ---
                if blue_count >= max_color_count and orange_count >= max_color_count:
                    print("Line crossing target reached, stopping robot.")
                    time.sleep(1)
                    robot_motion.robot_stop()
                    break

                # --- Navigation logic (unchanged) ---
                if scan_data:
                    error = calculate_steering_error(scan_data, target_distance_mm=target_distance_mm, safety_distance_mm=safety_distance_mm)
                    if error == 9999.0:
                        print("Obstacle detected! Stopping for safety.")
                        robot_motion.robot_stop()
                        time.sleep(1)
                        continue
                    pid_output = pid.update(error)
                    steering_angle = map_steering_angle(center_angle, pid_output, clockwise=clockwise_mode)
                    robot_motion.robot_forward_speed(0.90)
                    robot_motion.adjust_servo_angle(steering_angle)
                    print(f"Err: {error:.2f}, PID: {pid_output:.2f}, Servo: {steering_angle}, Blue: {blue_count}, Orange: {orange_count}")
                else:
                    print("No LiDAR data. Stopping.")
                    robot_motion.robot_stop()
                time.sleep(0.15)

    except KeyboardInterrupt:
        print("User interrupted.")
    finally:
        print("Shutting down...")
        picam2.close()
        robot_motion.robot_stop()
        robot_motion.motor_standby()
        print("Shutdown complete.")

if __name__ == "__main__":
    main()
