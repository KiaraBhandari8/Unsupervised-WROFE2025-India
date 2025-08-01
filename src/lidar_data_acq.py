import ydlidar
import math
import time

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


# --- Main execution block for testing ---
# This part runs only when you execute this file directly (e.g., `python lidar_scanner.py`).
# It demonstrates the intended usage of the LidarScanner class.
if __name__ == '__main__':
    print("Demonstrating LidarScanner class usage.")
    print("This will initialize the LiDAR, scan for 5 seconds, and then shut down.")
    print("Press Ctrl+C to exit early.")
    
    try:
        # Using a 'with' statement is the recommended approach.
        # It automatically handles calling the shutdown() method.
        with LidarScanner() as scanner:
            start_time = time.time()
            while time.time() - start_time < 5:
                lidar_points = scanner.get_scan_data()
                print(lidar_points)
                break

                if lidar_points:
                    # To avoid flooding the console, we'll just print a summary.
                    # In your main file, you would process this dictionary.
                    num_points = len(lidar_points)
                    # Find the point directly in front (0 degrees) if it exists
                    front_distance = lidar_points.get(0.0, -1) 
                    print(f"Scan successful: Received {num_points} points. Front distance: {front_distance:.2f}m")
                else:
                    print("Scan received no valid data points.")
                
                # A short sleep to prevent the loop from running too fast.
                time.sleep(0.1) 

    except (IOError, KeyboardInterrupt) as e:
        # Handles initialization errors or the user stopping the script.
        print(f"\nAn error occurred or program was stopped: {e}")
    
    print("\nDemonstration finished.")

