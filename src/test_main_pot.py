import time
import robot_motion
from lidar_steering_pot import LidarScanner, calculate_potential_field_steering

def map_steering_angle(center_angle, steering_error):
    """
    Maps the steering error from the potential field to the robot's servo angle.

    Args:
        center_angle (int): The servo angle for straight forward movement.
        steering_error (float): The calculated angle from the potential field.
                                Positive means turn left, negative means turn right.

    Returns:
        float: The calculated servo angle, clamped within safe limits.
    """
    # A positive error (turn left) should INCREASE the servo angle.
    # A negative error (turn right) should DECREASE the servo angle.
    # The scale factor adjusts how aggressively the robot turns. Tune this!
    scale_factor = 0.75
    
    # Positive error -> turn left -> increase angle
    angle = center_angle + (-1 * steering_error * scale_factor)

    min_angle = 70   # Right turn limit
    max_angle = 110  # Left turn limit
    return max(min_angle, min(angle, max_angle))

def main():
    print("=== Robot Potential Field Navigation Initialized ===")
    center_angle = 90

    try:
        with LidarScanner() as scanner:
            print("LiDAR is active.")

            while True:
                scan_data = scanner.get_scan_data()

                if scan_data:
                    # Calculate the desired steering direction using potential fields
                    steering_error = calculate_potential_field_steering(scan_data)
                    
                    # Map this direction to a physical servo angle
                    steering_angle = map_steering_angle(center_angle, steering_error)

                    # Move the robot
                    robot_motion.robot_forward_speed(0.75)
                    robot_motion.adjust_servo_angle(steering_angle)

                    print(f"Steering Error: {steering_error:.2f} deg, Servo Angle: {steering_angle:.2f}")
                else:
                    print("No LiDAR data. Stopping for safety.")
                    robot_motion.robot_stop()

                time.sleep(0.1) # A shorter delay for faster reaction time

    except IOError as e:
        print(f"ERROR: LiDAR issue: {e}")
    except KeyboardInterrupt:
        print("User interrupted.")
    finally:
        print("Shutdown initiated...")
        robot_motion.robot_stop()
        robot_motion.motor_standby()
        print("Shutdown complete.")

if __name__ == "__main__":
    main()