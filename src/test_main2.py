import time
import cv2
import numpy as np
import robot_motion_sample
from picamera2 import Picamera2
from libcamera import Transform
from lidar_steering1 import LidarScanner, PIDController, calculate_steering_error

# =============================================================================
# --- Camera and Color Detection Functions ---
# =============================================================================

def filter_green_objects(hsv_frame):
    """
    Processes an HSV image to find green objects and returns their contours.
    """
    lower_green = np.array([30, 40, 40])
    upper_green = np.array([85, 255, 255])
    green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
    kernel = np.ones((5, 5), np.uint8)
    green_mask_eroded = cv2.erode(green_mask, kernel, iterations=3)
    green_mask_dilated = cv2.dilate(green_mask_eroded, kernel, iterations=1)
    green_contours, _ = cv2.findContours(green_mask_dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return len(green_contours), green_contours

# =============================================================================
# --- Steering and Navigation Functions ---
# =============================================================================

def map_steering_angle(center_angle, pid_output, clockwise=True):
    """Maps PID output to a servo angle, clamping it within safe limits."""
    scale_factor = 1.4 if clockwise else 1.0
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
    print("=== Robot PID Navigation with Timed Avoidance State Machine Initialized ===")
    
    # --- Configuration ---
    clockwise_mode = True
    center_angle = 75.0
    robot_motion_sample.set_motor_speed(0.75)
    
    # --- PID Setup for LiDAR ---
    # NOTE: We assume the PIDController class has a reset() method.
    pid = PIDController(Kp=0.3, Ki=0.001, Kd=0.02) if clockwise_mode else PIDController(Kp=0.2, Ki=0.001, Kd=0.05)

    # --- State Machine Variables ---
    robot_state = 'WALL_FOLLOWING'
    state_timer_start = 0
    AVOIDANCE_DURATION_S = 1.0
    RECOVERY_DURATION_S = 1.0
    
    # --- Hardware Initialization ---
    picam2 = Picamera2()
    config = picam2.create_still_configuration(
        main={"size": (320, 240)},
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
                frame = picam2.capture_array()
                hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
                
                green_count, _ = filter_green_objects(hsv)
                
                # --- STATE MACHINE LOGIC ---
                
                if robot_state == 'WALL_FOLLOWING':
                    if green_count > 0:
                        # Obstacle detected, switch to AVOIDING state
                        robot_state = 'AVOIDING'
                        state_timer_start = time.time()
                        print("STATE CHANGE: WALL_FOLLOWING -> AVOIDING")
                    else:
                        # Default behavior: Use LiDAR for wall-following
                        scan_data = scanner.get_scan_data()
                        if scan_data:
                            error = calculate_steering_error(scan_data)
                            direction_error = -error if clockwise_mode else error
                            pid_output = pid.update(direction_error)
                            steering_angle = map_steering_angle(center_angle, pid_output, clockwise=clockwise_mode)
                        else:
                            print("No LiDAR data. Stopping for safety.")
                            robot_motion_sample.robot_stop()
                            time.sleep(0.5)
                            continue
                
                elif robot_state == 'AVOIDING':
                    # Perform a fixed left turn for a set duration
                    steering_angle = center_angle + 15
                    if time.time() - state_timer_start > AVOIDANCE_DURATION_S:
                        # Avoidance maneuver complete, switch to RECOVERING
                        robot_state = 'RECOVERING'
                        state_timer_start = time.time()
                        print("STATE CHANGE: AVOIDING -> RECOVERING")
                
                elif robot_state == 'RECOVERING':
                    # Perform a fixed right turn to straighten out
                    steering_angle = center_angle - 15 
                    if time.time() - state_timer_start > RECOVERY_DURATION_S:
                        # Recovery is complete, reset PID and go back to wall following
                        pid.reset() # IMPORTANT: Reset PID to prevent windup
                        robot_state = 'WALL_FOLLOWING'
                        print("STATE CHANGE: RECOVERING -> WALL_FOLLOWING (PID Reset)")

                # Clamp final steering angle
                final_steering_angle = max(50.0, min(steering_angle, 100.0))
                
                robot_motion_sample.robot_forward()
                robot_motion_sample.adjust_servo_angle(final_steering_angle)
                
                # --- Logging ---
                log_msg = f"State: {robot_state} | Servo: {final_steering_angle:.2f}"
                print(log_msg)
                
                time.sleep(0.1)

    except IOError as e:
        robot_motion_sample.robot_stop()
        print(f"ERROR: Hardware issue: {e}")
    except KeyboardInterrupt:
        robot_motion_sample.robot_stop()
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


# sudo /home/pi8/wrofe2025/env_test/bin/python /home/pi8/wrofe2025/main1.py