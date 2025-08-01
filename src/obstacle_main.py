import time
import cv2
import obstacle_robot_motion
from lidar_steering1 import LidarScanner, PIDController, calculate_steering_error
from obstacle_avoid import detect_pillar_color_and_position

def map_steering_angle(center_angle, pid_output, clockwise=True):
    scale_factor = 1.4 if clockwise else 1.0
    adjusted_output = pid_output * scale_factor
    angle = center_angle - adjusted_output if clockwise else center_angle + adjusted_output
    min_angle = 45.0
    max_angle = 105.0
    return max(min_angle, min(angle, max_angle))

def main():
    print("=== Robot Square Arena + Pillar Avoidance Navigation ===")
    clockwise_mode = True  # Set False for anticlockwise
    center_angle = 75.0

    # Robust camera open
    cap = None
    for idx in [0, 1, 2]:
        cap = cv2.VideoCapture(idx)
        if cap.isOpened():
            print(f"Camera opened at index {idx}")
            break
        cap.release()
    else:
        print("ERROR: Camera could not be opened. Exiting.")
        return

    obstacle_robot_motion.set_motor_speed(0.9)

    pid = PIDController(Kp=0.3, Ki=0.001, Kd=0.02) if clockwise_mode else PIDController(Kp=0.2, Ki=0.001, Kd=0.05)

    # State for pillar avoidance/correction
    avoid_in_progress = False
    correction_in_progress = False
    avoid_direction = None
    avoid_start_time = None
    correction_start_time = None
    last_avoid_was = None
    AVOID_TIME = 0.7
    CORRECT_TIME = 0.5

    try:
        with LidarScanner() as scanner:
            print("LiDAR is active.")
            print(f"Direction: {'Clockwise' if clockwise_mode else 'Anticlockwise'}")

            while True:
                # Camera input and pillar check
                ret, frame = cap.read()
                color, cx = (None, None)
                if ret:
                    color, cx = detect_pillar_color_and_position(frame)
                    cv2.imshow("Camera View", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        print("Shutdown requested via keyboard.")
                        break
                else:
                    print("No camera frame. Skipping iteration.")
                    time.sleep(0.1)
                    continue

                # Pillar avoidance/correction handling (priority over corners)
                if avoid_in_progress:
                    if avoid_direction == 'left':
                        obstacle_robot_motion.adjust_servo_angle(90)
                    elif avoid_direction == 'right':
                        obstacle_robot_motion.adjust_servo_angle(60)
                    obstacle_robot_motion.robot_forward()
                    if time.time() > avoid_start_time + AVOID_TIME:
                        avoid_in_progress = False
                        correction_in_progress = True
                        correction_start_time = time.time()
                elif correction_in_progress:
                    if last_avoid_was == 'left':
                        obstacle_robot_motion.adjust_servo_angle(60)
                    elif last_avoid_was == 'right':
                        obstacle_robot_motion.adjust_servo_angle(90)
                    obstacle_robot_motion.robot_forward()
                    if time.time() > correction_start_time + CORRECT_TIME:
                        correction_in_progress = False
                elif color == 'green':
                    print("Green pillar detected. Avoid left, then correct.")
                    avoid_in_progress = True
                    avoid_direction = 'left'
                    avoid_start_time = time.time()
                    last_avoid_was = 'left'
                    obstacle_robot_motion.adjust_servo_angle(90)
                    obstacle_robot_motion.robot_forward()
                elif color == 'red':
                    print("Red pillar detected. Avoid right, then correct.")
                    avoid_in_progress = True
                    avoid_direction = 'right'
                    avoid_start_time = time.time()
                    last_avoid_was = 'right'
                    obstacle_robot_motion.adjust_servo_angle(60)
                    obstacle_robot_motion.robot_forward()
                else:
                    # Default: LiDAR PID navigation for square/corners
                    scan_data = scanner.get_scan_data()
                    if scan_data:
                        error = calculate_steering_error(scan_data)
                        direction_error = -error if clockwise_mode else error
                        pid_output = pid.update(direction_error)
                        steering_angle = map_steering_angle(center_angle, pid_output, clockwise=clockwise_mode)
                        obstacle_robot_motion.robot_forward()
                        obstacle_robot_motion.adjust_servo_angle(steering_angle)
                        print(f"PID: {pid_output:.2f}, Servo Angle: {steering_angle:.2f}")
                    else:
                        print("No LiDAR data. Stopping for safety.")
                        obstacle_robot_motion.robot_stop()
                time.sleep(0.1)

    except IOError as e:
        print(f"ERROR: LiDAR issue: {e}")
    except KeyboardInterrupt:
        print("User interrupted.")
    finally:
        print("Shutdown initiated...")
        if cap:
            cap.release()
            cv2.destroyAllWindows()
        obstacle_robot_motion.robot_stop()
        obstacle_robot_motion.motor_standby()
        print("Shutdown complete.")

if __name__ == "__main__":
    main()
