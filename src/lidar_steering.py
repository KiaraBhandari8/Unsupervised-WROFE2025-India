import ydlidar
import math
import time

class PIDController:
    """A simple PID controller."""
    def __init__(self, Kp, Ki, Kd):
        """
        Initializes the PID controller with gains.
        Args:
            Kp (float): Proportional gain.
            Ki (float): Integral gain.
            Kd (float): Derivative gain.
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def update(self, error):
        """
        Calculates the PID output.
        Args:
            error (float): The current error term (e.g., right_sum - left_sum).
        Returns:
            float: The PID output value (steering adjustment).
        """
        current_time = time.time()
        dt = current_time - self.last_time
        if dt == 0:
            return 0 # Avoid division by zero

        # Proportional term
        P = self.Kp * error

        # Integral term
        self.integral += error * dt
        I = self.Ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / dt
        D = self.Kd * derivative

        # Update state for next iteration
        self.prev_error = error
        self.last_time = current_time

        return P + I + D

class LidarScanner:
    """
    A class to manage the YDLIDAR T-mini Plus lifecycle and provide scan data.

    This class handles the one-time setup (initialization, configuration) and
    provides a method to repeatedly fetch scan data efficiently. It ensures
    proper shutdown of the LiDAR sensor using a `shutdown()` method or by
    acting as a context manager in a `with` statement.
    """
    def __init__(self, port="/dev/ttyUSB0"):
        """
        Initializes, configures, and turns on the LiDAR sensor.
        This performs the entire one-time setup.
        
        Args:
            port (str): The serial port the LiDAR is connected to.
        
        Raises:
            IOError: If the LiDAR fails to initialize or turn on.
        """
        # --- LiDAR Configuration ---
        self.port = port
        self.baud_rate = 230400
        self.MIN_ANGLE = -180.0
        self.MAX_ANGLE = 180.0
        self.MIN_RANGE = 0.02  # Min valid distance in meters
        self.MAX_RANGE = 12.0  # Max valid distance in meters

        # Initialize the LiDAR system signal handler
        ydlidar.os_init()

        # Create a Lidar object
        self.laser = ydlidar.CYdLidar()

        # --- Set LiDAR Properties ---
        self.laser.setlidaropt(ydlidar.LidarPropSerialPort, self.port)
        self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, self.baud_rate)
        self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
        self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 4)
        self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
        self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, True)
        
        # Set angle and range limits
        self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, self.MAX_ANGLE)
        self.laser.setlidaropt(ydlidar.LidarPropMinAngle, self.MIN_ANGLE)
        self.laser.setlidaropt(ydlidar.LidarPropMaxRange, self.MAX_RANGE)
        self.laser.setlidaropt(ydlidar.LidarPropMinRange, self.MIN_RANGE)

        # Initialize the LiDAR
        ret = self.laser.initialize()
        if not ret:
            raise IOError(f"Failed to initialize YDLIDAR: {self.laser.DescribeError()}")

        # Turn on the LiDAR for scanning
        ret = self.laser.turnOn()
        if not ret:
            self.laser.disconnecting()
            raise IOError(f"Failed to turn on YDLIDAR: {self.laser.DescribeError()}")
            
        print("YDLIDAR initialized and turned on successfully.")

    def get_scan_data(self):
        """
        Performs a single 360-degree scan and returns the processed data.
        This function is designed to be called repeatedly and efficiently.

        Returns:
            dict: A dictionary where keys are angles in degrees (float)
                  and values are corresponding distances in meters (float).
                  Returns an empty dictionary if the scan fails to get points.
        """
        scan = ydlidar.LaserScan()
        scan_data = {}
        
        # Perform one full 360-degree scan
        r = self.laser.doProcessSimple(scan)
        
        if r and len(scan.points) > 0:
            for p in scan.points:
                # The SDK and hardware settings should filter this, but it's a good safeguard.
                if self.MIN_RANGE <= p.range <= self.MAX_RANGE:
                    # Convert angle from radians to degrees for the dictionary key
                    angle_degrees = math.degrees(p.angle)
                    scan_data[angle_degrees] = p.range

        return scan_data

    def shutdown(self):
        """
        Turns off and disconnects the LiDAR sensor, releasing system resources.
        It's critical to call this when you are finished with the sensor.
        """
        print("Shutting down LiDAR...")
        self.laser.turnOff()
        self.laser.disconnecting()
        print("LiDAR has been turned off and disconnected.")

    def __enter__(self):
        """Allows the class to be used in a `with` statement for automatic cleanup."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Ensures shutdown() is called when exiting a `with` block."""
        self.shutdown()

def calculate_steering_angle(scan_data, pid_controller):
    """
    Calculates a steering angle based on LiDAR data using a PID controller.
    
    Args:
        scan_data (dict): The dictionary of angle-distance pairs from the LiDAR.
        pid_controller (PIDController): An instance of the PID controller.
        
    Returns:
        float: The calculated steering angle. Positive values steer right,
               negative values steer left.
    """
    if not scan_data:
        return 0.0 # No data, no turn
# 1. Separate points into left and right lists of (angle, distance) tuples
    left_points = sorted(
        [(a, d) for a, d in scan_data.items() if -90 <= a < 0],
        key=lambda x: x[0], reverse=True  # Sorts from -0.1 down to -90
    )
    right_points = sorted(
        [(a, d) for a, d in scan_data.items() if 0 <= a <= 90],
        key=lambda x: x[0]  # Sorts from 0.1 up to 90
    )

    # 2. Determine the minimum number of points to use for a fair comparison
    num_points_to_use = min(len(left_points), len(right_points))

    if num_points_to_use == 0:
        # If one side is completely empty, we can't make a fair comparison.
        # A more advanced strategy could be implemented here (e.g., turn away
        # from the side that still has points), but for now, we don't steer.
        return 0.0

    # 3. Get the distances from the balanced number of points (closest to front)
    left_distances = [p[1] for p in left_points[:num_points_to_use]]
    right_distances = [p[1] for p in right_points[:num_points_to_use]]
    
    # 4. Calculate the sum of distances for each side
    left_dist_sum = sum(left_distances)
    right_dist_sum = sum(right_distances)

    # Use the sum of distances as a simple metric for how "clear" a side is.
    # A higher sum means more open space.
    left_dist_sum = sum(left_distances) if left_distances else 0
    right_dist_sum = sum(right_distances) if right_distances else 0

    # The error is the difference between the right and left sides.
    # A positive error means the right side is more open, so we should steer right.
    # A negative error means the left side is more open, so we should steer left.
    error = right_dist_sum - left_dist_sum
    
    # Get the steering adjustment from the PID controller
    steering_adjustment = pid_controller.update(error)

    # Clamp the output to a reasonable range, e.g., -45 to 45 degrees
    max_steer = 45.0
    steering_angle = max(-max_steer, min(steering_adjustment, max_steer))
    
    print(f"L_Sum: {left_dist_sum:.2f}, R_Sum: {right_dist_sum:.2f}, Error: {error:.2f}, Steer_Angle: {steering_angle:.2f}")

    return steering_angle

# --- Main execution block for testing ---
if __name__ == '__main__':
    print("Demonstrating LidarScanner and PID steering calculation.")
    print("Press Ctrl+C to exit.")
    
    # --- PID Tuning: These values will need to be adjusted for your specific robot ---
    # Kp: Proportional - How strongly to react to the current error.
    # Ki: Integral - Corrects for steady-state error over time.
    # Kd: Derivative - Dampens overshoot and oscillations.
    pid = PIDController(Kp=0.01, Ki=0.001, Kd=0.005)

    try:
        with LidarScanner() as scanner:
            while True:
                # 1. Get scan data
                lidar_points = scanner.get_scan_data()

                if lidar_points:
                    # 2. Calculate steering angle using the data and PID controller
                    steer = calculate_steering_angle(lidar_points, pid)
                    # In a real robot, you would now send this 'steer' value to your motor controller.
                else:
                    print("Scan received no valid data points.")
                
                time.sleep(0.1) # Loop delay

    except (IOError, KeyboardInterrupt) as e:
        print(f"\nAn error occurred or program was stopped: {e}")
    
    print("\nDemonstration finished.")