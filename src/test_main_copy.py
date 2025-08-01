import time
import robot_motion
from lidar_steering2 import LidarScanner, PIDController, calculate_steering_error

def map_steering_angle(center_angle, pid_output, clockwise=True):
    """
    Maps the PID output to the robot's steering servo angle.

    Args:
        center_angle (int): The servo angle that corresponds to straight forward movement.
        pid_output (float): The output from the PID controller.
        clockwise (bool): True if the robot is navigating in a clockwise direction (right wall following).

    Returns:
        float: The calculated servo angle, clamped within the safe operational range.
    """
    # Adjust scale factor based on observed robot behavior and responsiveness
    # You might need to fine-tune these based on physical testing
    scale_factor = 0.40 # Start with 1.0 and adjust as needed

    adjusted_output = pid_output * scale_factor

    if clockwise:
        # For clockwise (right wall following), positive PID output means move away from the right wall (steer left)
        # which means increasing the servo angle from the center.
        # However, your error calculation (avg_right_distance - target_distance_mm) gives positive when too far.
        # If too far from right wall, need to steer RIGHT (decrease angle).
        # So, positive PID output from "too far" should decrease angle.
        # If too close to right wall, need to steer LEFT (increase angle).
        # So, negative PID output from "too close" should increase angle.
        # This implies a subtraction for clockwise.
        angle = center_angle - adjusted_output
    else:
        # For anticlockwise (left wall following), positive PID output means move away from the left wall (steer right)
        # which means increasing the servo angle from the center.
        # Your error for left wall is -(avg_left_distance - target_distance_mm).
        # If too far from left wall, (avg_left_distance - target) is positive, error is negative. Need to steer LEFT (decrease angle).
        # So, negative PID output should decrease angle.
        # If too close to left wall, (avg_left_distance - target) is negative, error is positive. Need to steer RIGHT (increase angle).
        # So, positive PID output should increase angle.
        # This implies an addition for anticlockwise.
        angle = center_angle + adjusted_output

    # --- NEW SERVO ANGLE RANGES ---
    min_angle = 70   # Right turn limit
    max_angle = 110  # Left turn limit
    return int(max(min_angle, min(angle, max_angle)))

def main():
    print("=== Robot PID Navigation Initialized ===")
    clockwise_mode = True     # Set False for anticlockwise
    
    # --- NEW SERVO CENTER ANGLE ---
    center_angle = 90

    # Explicitly set global robot speed to 90% (this line was commented, uncomment if you want to enforce it)
    # robot_motion.set_motor_speed(0.9) # Assuming 1.0 is max, 0.9 is 90%

    if clockwise_mode:
        # PID values might need re-tuning for the new servo range and robot dynamics
        # pid = PIDController(Kp=0.3, Ki=0.001, Kd=0.02, setpoint=0) 
        pid = PIDController(Kp=0.3, Ki=0.001, Kd=0.02, setpoint=0) 

        # setpoint should ideally be 0 for error
    else:
        # PID values might need re-tuning for the new servo range and robot dynamics
        # pid = PIDController(Kp=0.2, Ki=0.001, Kd=0.05, setpoint=0)
        pid = PIDController(Kp=0.2, Ki=0.001, Kd=0.05, setpoint=0)

         # setpoint should ideally be 0 for error
    start_time = time.time()
    try:
        with LidarScanner() as scanner:
            print("LiDAR is active.")
            print(f"Direction: {'Clockwise' if clockwise_mode else 'Anticlockwise'}")

            while True:
                scan_data = scanner.get_scan_data()
                elapsed_time = time.time() - start_time
                if scan_data:
                    error = calculate_steering_error(scan_data, target_distance_mm=750, safety_distance_mm=300)
                    
                    # If calculate_steering_error returns a large error for obstacle, handle it
                    if error == 9999.0:
                        print("Emergency stop due to obstacle!")
                        robot_motion.robot_stop()
                        time.sleep(1) # Pause before trying to resume
                        continue # Skip to next loop iteration

                    # The `calculate_steering_error` function already provides the error sign
                    # consistent with "turn left for negative error, turn right for positive error"
                    # if we consider positive error means "too far from the wall you're following"
                    # For clockwise (right wall following):
                    #   - error > 0 (too far from right wall) -> need to steer RIGHT (decrease angle).
                    #   - error < 0 (too close to right wall) -> need to steer LEFT (increase angle).
                    # This means the pid_output should be SUBTRACTED from center_angle if the PID output
                    # for a positive error is positive.

                    # For anticlockwise (left wall following):
                    #   - error > 0 (too far from left wall, which means robot is too far RIGHT of path) -> need to steer LEFT (decrease angle)
                    #   - error < 0 (too close to left wall, which means robot is too far LEFT of path) -> need to steer RIGHT (increase angle)
                    # This means the pid_output should be ADDED to center_angle if the PID output
                    # for a positive error is positive.

                    # Given calculate_steering_error returns `avg_right_distance - target_distance_mm`
                    # for right wall (clockwise):
                    #   - If right_dist > target (error positive), robot is too far right, needs to steer RIGHT.
                    #     PID will output positive. `center - PID_output` will steer right. This is correct.
                    # Given calculate_steering_error returns `-(avg_left_distance - target_distance_mm)`
                    # for left wall (anticlockwise):
                    #   - If left_dist > target (`(left-target)` positive), error is negative. Robot is too far left, needs to steer LEFT.
                    #     PID will output negative. `center + PID_output` (effectively `center - abs(PID_output)`) will steer left. This is correct.

                    # So the `direction_error` negation is not strictly needed if `calculate_steering_error`
                    # already handles the directionality for wall following.
                    # Let's use the error directly, and tune the map_steering_angle for the servo direction.
                    
                    pid_output = pid.update(error) # Use the error directly from calculate_steering_error

                    steering_angle = map_steering_angle(center_angle, pid_output, clockwise=clockwise_mode)

                    # Maintain speed at every iteration for stability
                    robot_motion.robot_forward_speed(0.90)
                    robot_motion.adjust_servo_angle(steering_angle)

                    print(f"Error: {error:.2f}, PID: {pid_output:.2f}, Servo Angle: {steering_angle:.2f}")
                    print(f"Time Elapsed : {elapsed_time:.2f}s")
                else:
                    print("No LiDAR data. Stopping for safety.")
                    robot_motion.robot_stop()

                time.sleep(0.15)

    except IOError as e:
        print(f"ERROR: LiDAR issue: {e}")
        # robot_motion.robot_stop()
    except KeyboardInterrupt:
        print("User interrupted.")
        # robot_motion.robot_stop()
    finally:
        print("Shutdown initiated...")
        robot_motion.robot_stop()
        robot_motion.motor_standby()
        print("Shutdown complete.")

if __name__ == "__main__":
    main()