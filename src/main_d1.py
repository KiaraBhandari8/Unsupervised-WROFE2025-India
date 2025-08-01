# main_d1.py

import time
import sys

# Import functions from robot_motion.py
try:
    from robot_motion import (
        robot_forward,
        robot_stop,
        robot_left_90,
        robot_right_90,
        adjust_servo_angle,
        motor_standby,
        current_servo_angle # Import current_servo_angle for initial setup
    )
except ImportError:
    print("Error: robot_motion.py not found. Please ensure it's in the same directory.")
    sys.exit(1)

# Import functions and GPIO/sensor setup from ultrasonic.py
try:
    from ultrasonic import (
        get_sensor_distances,
        get_state_angle,
        GPIO, # Need GPIO for cleanup
        sensors # Need sensors list for initial setup in ultrasonic.py
    )
except ImportError:
    print("Error: ultrasonic.py not found. Please ensure it's in the same directory.")
    sys.exit(1)

# --- Configuration and Tuning Parameters ---
# These values will likely need significant tuning for your specific robot and environment.
# Servo sensitivity for steering adjustments when 'Between Lines'
SERVO_STEERING_SENSITIVITY = 1.0 # Adjust this (e.g., 0.2 to 0.5) for how aggressively the robot steers.
                                 # A higher value means more aggressive steering.
STRAIGHT_SERVO_ANGLE = 75        # The servo angle that makes the robot go straight.

# Delay after turns to allow robot to settle and sensors to get new readings
POST_TURN_SETTLE_TIME = 0.5

# Main loop delay to control how frequently the robot checks sensors and updates motion
LOOP_DELAY = 0.05

# --- Main Autonomous Navigation Logic ---
if __name__ == "__main__":
    # Initial setup: Ensure robot is stopped and servo is in a neutral position.
    # These functions are called from robot_motion.py, which handles its own GPIO setup.
    print("Initializing robot...")
    robot_stop()
    adjust_servo_angle(STRAIGHT_SERVO_ANGLE) # Set servo to go straight
    time.sleep(1) # Short delay for stability

    try:
        print("Starting autonomous navigation...")
        robot_forward() # Start moving forward initially

        while True:
            # Get current state and suggested angle from ultrasonic sensors
            # get_state_angle also calls get_sensor_distances internally.
            state, angle, left_d, right_d, forward_d = get_state_angle()

            print(f"L:{left_d}cm, R:{right_d}cm, F:{forward_d}cm | State: {state}, Angle: {angle}")

            if state == 'BL': # Between Lines - Adjust steering
                # Ensure the robot is moving forward if it's in the 'BL' state
                # (in case it stopped due to a previous 'None' state or just started)
                robot_forward()

                if angle is not None:
                    # Calculate the desired servo angle based on the 'angle' provided by get_state_angle.
                    # 'angle' from ultrasonic.py is (ld-rd)*maxangle/225, ranging from -45 to 45.
                    # A positive 'angle' means the robot is closer to the right wall, needs to steer left (increase servo angle).
                    # A negative 'angle' means the robot is closer to the left wall, needs to steer right (decrease servo angle).
                    # 'STRAIGHT_SERVO_ANGLE' is the base.
                    desired_servo_angle = STRAIGHT_SERVO_ANGLE + (angle * SERVO_STEERING_SENSITIVITY)
                    adjust_servo_angle(int(desired_servo_angle)) # Set the servo angle

            elif state == 'LC': # Left Corner - Robot needs to turn left 90 degrees
                print("Detected Left Corner. Executing Left 90-degree turn.")
                robot_stop() # Stop before turning
                time.sleep(0.1) # Small pause for stability
                robot_left_90() # This function includes its own sleep for the turn duration
                robot_forward() # Continue moving forward after the turn
                time.sleep(POST_TURN_SETTLE_TIME) # Give it a moment to stabilize and get new readings

            elif state == 'RC': # Right Corner - Robot needs to turn right 90 degrees
                print("Detected Right Corner. Executing Right 90-degree turn.")
                robot_stop() # Stop before turning
                time.sleep(0.1) # Small pause for stability
                robot_right_90() # This function includes its own sleep for the turn duration
                robot_forward() # Continue moving forward after the turn
                time.sleep(POST_TURN_SETTLE_TIME) # Give it a moment to stabilize and get new readings

            else: # state is None (Lost/Open Space or unknown state)
                print("Robot: Unknown state or lost walls. Stopping.")
                robot_stop()
                # You might add a search pattern here, or simply stop and wait for intervention.
                # For now, we'll just stop and wait.
                time.sleep(1) # Prevent rapid state changes if it's oscillating around 'None'

            time.sleep(LOOP_DELAY) # Small delay to control loop frequency

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Stopping robot and cleaning up GPIO...")
    except Exception as e:
        print(f"\nAn unexpected error occurred: {e}")
    finally:
        # Ensure all robot components are safely shut down
        robot_stop()
        motor_standby() # Put motor driver in standby mode
        adjust_servo_angle(STRAIGHT_SERVO_ANGLE) # Reset servo to neutral position

        # IMPORTANT: Clean up GPIO pins.
        # Since ultrasonic.py sets up GPIO, we need to call its cleanup.
        # The GPIO.cleanup() function from RPi.GPIO (imported via ultrasonic.py)
        # will clean up all pins that were set up by RPi.GPIO.
        GPIO.cleanup()
        print("GPIO cleanup complete. Exiting.")