import time
import robot_motion
# Import the classes and function from your lidar_steering module
from lidar_steering import LidarScanner, PIDController, calculate_steering_angle

def main():
    """
    Main function to run the robot's autonomous navigation loop.
    """
    print("Initializing robot systems...")
    
    # --- One-Time Initialization ---
    # These objects are created once, outside the main loop, for efficiency.
    
    # Initialize the PID controller with your tuned gains.
    # Kp: Proportional - How strongly to react to the current error.
    # Ki: Integral - Corrects for steady-state error over time.
    # Kd: Derivative - Dampens overshoot and oscillations.
    pid = PIDController(Kp=0.3, Ki=0.001, Kd=0.05)

    # Use a 'with' statement for the LidarScanner.
    # This automatically handles initialization and ensures the LiDAR
    # is properly shut down when the block is exited (e.g., on error or Ctrl+C).
    with LidarScanner() as scanner:
        print("LiDAR is active. Starting navigation loop...")
        print("Press Ctrl+C to stop the robot.")
        
        # --- Main Navigation Loop ---
        while True:
            # 1. Get the latest scan data from the LiDAR.
            lidar_points = scanner.get_scan_data()

            if lidar_points:
                # 2. Calculate the required steering adjustment.
                # The PID controller's state is preserved between calls.
                steer_adjustment = calculate_steering_angle(lidar_points, pid)
                
                # 3. Convert the adjustment to a final servo angle.
                # Assuming 75 degrees is straight ahead.
                # A positive adjustment from PID means steer right (e.g., 75 - 10 = 65).
                # A negative adjustment from PID means steer left (e.g., 75 - (-10) = 85).
                center_angle = 75.0
                final_steering_angle = center_angle - steer_adjustment
                
                # Clamp the final angle to the servo's physical limits (e.g., 30 to 120)
                # to prevent damage. Adjust these values as needed for your hardware.
                min_servo_angle = 50.0
                max_servo_angle = 100.0
                final_steering_angle = max(min_servo_angle, min(final_steering_angle, max_servo_angle))

                # 4. Command the robot.
                robot_motion.robot_forward()
                robot_motion.adjust_servo_angle(final_steering_angle)
                print(f"Final Steering Angle : {final_steering_angle}")
            else:
                # If no data is received, it's safer to stop.
                print("No LiDAR data received. Stopping robot.")
                robot_motion.robot_stop()

            # Loop delay to control the update frequency.
            time.sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    except IOError as e:
        # This will catch initialization errors from the LidarScanner.
        print(f"\nA critical hardware error occurred: {e}")
    except KeyboardInterrupt:
        # This allows the user to stop the robot cleanly with Ctrl+C.
        print("\nProgram stopped by user.")
    finally:
        # Ensure the robot is stopped and in standby on exit.
        print("Executing final shutdown.")
        robot_motion.robot_stop()
        robot_motion.motor_standby()

