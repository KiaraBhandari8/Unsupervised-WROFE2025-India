import os
os.environ["GPIOZERO_PIN_FACTORY"] = "lgpio"

import time
import cv2 # cv2 is still imported but not used, you can remove it if you wish
import obstacle_robot_motion
from lidar_steering1 import LidarScanner, PIDController, calculate_steering_error
# from obstacle_avoid import detect_pillar_color_and_position # Removed obstacle_avoid import

def map_steering_angle(center_angle, pid_output, clockwise=True):
    # Reverting scale_factor to previous values
    scale_factor = 1.4 if clockwise else 1.0
    adjusted_output = pid_output * scale_factor
    angle = center_angle - adjusted_output if clockwise else center_angle + adjusted_output
    min_angle = 45.0
    max_angle = 105.0
    return max(min_angle, min(angle, max_angle))

def main():
    print("=== Robot Square Arena Navigation (No Pillar Avoidance) ===")
    clockwise_mode = True  # Set False for anticlockwise
    center_angle = 90.0 # This was 75.0 in your previous main, I've kept 90.0 as it was in your copy-pasted input.

    # --- Configuration ---
    LOOP_SLEEP_TIME = 0.1

    # Camera related lines are removed from here as they are no longer needed
    # for pillar detection. The cv2 import at the top can be removed too if desired.

    obstacle_robot_motion.set_motor_speed(0.9)

    # --- PID Controller Gains: Reverted to previous values ---
    if clockwise_mode:
        pid = PIDController(Kp=0.3, Ki=0.001, Kd=0.02)
    else:
        pid = PIDController(Kp=0.2, Ki=0.001, Kd=0.05)

    # All pillar avoidance state variables are removed as they are no longer relevant

    try:
        with LidarScanner() as scanner:
            print("LiDAR is active.")
            print(f"Direction: {'Clockwise' if clockwise_mode else 'Anticlockwise'}")

            while True:
                # Removed all camera input and pillar check logic
                # color, cx = (None, None) # This is no longer needed

                # Default: LiDAR PID navigation for square/corners
                scan_data = scanner.get_scan_data()
                if scan_data:
                    error = calculate_steering_error(scan_data)
                    direction_error = -error if clockwise_mode else error
                    pid_output = pid.update(direction_error)
                    steering_angle = map_steering_angle(center_angle, pid_output, clockwise=clockwise_mode)
                    obstacle_robot_motion.robot_forward()
                    obstacle_robot_motion.adjust_servo_angle(steering_angle)
                else:
                    print("No LiDAR data. Stopping for safety.")
                    obstacle_robot_motion.robot_stop()
                time.sleep(LOOP_SLEEP_TIME)

    except IOError as e:
        print(f"ERROR: LiDAR issue: {e}")
    except KeyboardInterrupt:
        print("User interrupted.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        print("Shutdown initiated...")
        # Removed camera release and destroyAllWindows as camera is no longer active
        obstacle_robot_motion.robot_stop()
        obstacle_robot_motion.motor_standby()
        print("Shutdown complete.")

if __name__ == "__main__":
    main()