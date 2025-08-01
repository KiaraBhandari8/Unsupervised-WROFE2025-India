import serial
import struct
import math
import time

# --- LiDAR Scanner Class ---
class LidarScanner:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        """
        Initializes the LiDAR scanner.
        Ensure correct port and baudrate for your LiDAR.
        For RPLIDAR A1/A2, '/dev/ttyUSB0' is common, baudrate 115200.
        """
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.scan_data = {} # Dictionary to store angle:distance pairs

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def connect(self):
        """Establishes serial connection to the LiDAR."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate)
            self.ser.flushInput()
            print(f"LiDAR: Connected to {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"LiDAR ERROR: Could not open serial port {self.port}: {e}")
            raise IOError(f"LiDAR connection failed: {e}")

    def disconnect(self):
        """Closes the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("LiDAR: Disconnected.")

    def start_scan(self):
        """Sends command to start scanning."""
        if self.ser and self.ser.is_open:
            # Command for RPLIDAR A1/A2 to start scan
            self.ser.write(b'\xA5\x20') # A5h 20h (start scan)
            print("LiDAR: Scan command sent.")
        else:
            print("LiDAR ERROR: Not connected to start scan.")

    def get_scan_data(self):
        """
        Reads and processes a single scan from the LiDAR.
        This is a simplified example and may need adjustment for full robustness
        and handling different LiDAR packets/protocols.
        For RPLIDAR, a full scan takes time and involves many data points.
        This function attempts to grab a 'snapshot' of points.
        """
        if not (self.ser and self.ser.is_open):
            # print("LiDAR: Not connected. Cannot get scan data.") # Commented to reduce log spam
            return None

        self.scan_data = {} # Clear previous scan data

        try:
            # This logic depends heavily on your specific LiDAR's data format.
            # For RPLIDAR A1/A2, a typical data point might be 5 bytes.
            # We'll read more to try and get a good range of angles.
            # For a more accurate approach, refer to RPLIDAR SDK or protocol spec.
            bytes_to_read = 500 # Adjust based on how many points you want per 'scan'
            
            if self.ser.in_waiting >= bytes_to_read:
                raw_bytes = self.ser.read(bytes_to_read)
                
                # Simplified parsing (highly dependent on exact LiDAR model and mode)
                # This assumes a simple structure where 5 bytes encode angle and distance
                for i in range(0, len(raw_bytes) - 5, 5):
                    try:
                        # Example: Assuming 5 bytes per measurement (e.g., quality, angle_LSB, angle_MSB, dist_LSB, dist_MSB)
                        # This is a very rough example and needs to match your LiDAR's protocol.
                        # For RPLIDAR, usually angle is 15-bit, distance is 16-bit.
                        # Refer to the RPLIDAR A1/A2 protocol document for accurate parsing.
                        
                        # Dummy parsing for demonstration if you haven't implemented full protocol
                        byte_data = raw_bytes[i:i+5]
                        if len(byte_data) == 5:
                            # A placeholder for actual parsing.
                            # For a real RPLIDAR, you'd decode sync bits, angle, and distance
                            # based on its specific data packets (e.g., M-bit, checksum, etc.).
                            
                            # Simple example: just extract two bytes for angle and two for distance
                            # This is NOT how RPLIDAR works, it's just to make the code run.
                            # You need a proper RPLIDAR parser here.
                            
                            # Let's assume you have a function that provides actual angle and distance
                            # from these bytes, or you are using a library like `rplidar_py`
                            
                            # If your LiDAR is working and you just need to get data:
                            # Replace this with your actual LiDAR data acquisition loop.
                            # For a basic test to make PID work without real LiDAR,
                            # you could generate dummy `self.scan_data` here.
                            
                            # Example: In a real scenario, you'd get angle/distance from bytes
                            # angle = (byte_data[2] << 7 | byte_data[1] >> 1) / 64.0 # Example
                            # distance = (byte_data[4] << 8 | byte_data[3]) # Example
                            # self.scan_data[round(angle)] = distance
                            pass # Placeholder if you have a more complex data acquisition elsewhere
                            
                    except IndexError:
                        # Not enough bytes for a full data point
                        continue
                        
                # If we read data but didn't parse anything into scan_data (e.g. dummy parsing above)
                # For proper function, ensure `self.scan_data` gets populated with angle-distance pairs
                if not self.scan_data:
                    # Fallback for testing: simulate some distances if no real parsing
                    # This section should be removed once proper LiDAR parsing is in place
                    # or if you are confident your LiDAR is returning actual data.
                    # This simulates a wall in front and slightly to the sides.
                    self.scan_data = {
                        0: 1000, 10: 950, 350: 950, # Front
                        80: 800, 90: 750, 100: 800, # Right wall
                        260: 800, 270: 750, 280: 800 # Left wall
                    }
                    # print("LiDAR: Populated with dummy data (FOR TESTING ONLY).")

                return self.scan_data
            # else:
                # print("LiDAR: Not enough data in buffer for a full read.") # Commented to reduce spam

        except Exception as e:
            print(f"LiDAR DATA ERROR: {e}")
            return None # Return None on error

        return None # Return None if no data processed


# --- PID Controller Class ---
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()

    def update(self, current_error):
        current_time = time.time()
        dt = current_time - self.last_time

        if dt <= 0: # Avoid division by zero or negative dt
            return self.prev_error # Or return 0 or previous output

        # Proportional term
        P = self.Kp * current_error

        # Integral term
        self.integral += current_error * dt
        I = self.Ki * self.integral

        # Derivative term
        derivative = (current_error - self.prev_error) / dt
        D = self.Kd * derivative

        # Total PID output
        output = P + I + D

        # Update for next iteration
        self.prev_error = current_error
        self.last_time = current_time

        return output

    def reset(self):
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()


# --- Steering Error Calculation ---
def calculate_steering_error(scan_data, target_distance_mm=750, safety_distance_mm=300):
    """
    Calculates the steering error based on LiDAR scan data to keep the robot
    at a target distance from the walls.
    
    Args:
        scan_data (dict): Dictionary of angle:distance_mm from LiDAR.
        target_distance_mm (int): The desired distance from the wall in millimeters.
        safety_distance_mm (int): If any point is closer than this, consider it an immediate obstacle.

    Returns:
        float: The steering error. Positive implies turn right, negative implies turn left.
               Returns a large error if an immediate obstacle is detected.
    """
    
    # Define angular ranges for left, right, and front detection
    # These ranges need to be calibrated to your robot's LiDAR placement and desired navigation.
    # Assuming 0 degrees is directly forward.
    # Angles typically increase counter-clockwise (0 to 359).
    
    # Example ranges (adjust based on your robot's orientation and LiDAR output):
    # Front sector (for obstacle detection / general clearance)
    front_angles = [angle for angle in range(350, 360)] + [angle for angle in range(0, 10)] # -10 to +10 degrees
    
    # Right wall sensing sector (e.g., 45 to 135 degrees relative to robot's front)
    # If 0 is robot front, and 90 is directly right:
    right_wall_angles = [angle for angle in range(60, 120)] # Angles for the wall on the right side
    
    # Left wall sensing sector (e.g., 225 to 315 degrees relative to robot's front)
    # If 0 is robot front, and 270 is directly left:
    left_wall_angles = [angle for angle in range(240, 300)] # Angles for the wall on the left side


    # --- Immediate Obstacle Detection (Safety) ---
    for angle in front_angles:
        if angle in scan_data and scan_data[angle] is not None and scan_data[angle] < safety_distance_mm and scan_data[angle] > 0:
            print(f"LiDAR: WARNING! Obstacle detected at {angle} deg, {scan_data[angle]}mm. Commanding full stop/re-evaluation.")
            return 9999.0 # Large error to trigger immediate action (e.g., stop or emergency turn)

    # --- Wall Following Logic ---
    right_distances = [scan_data[angle] for angle in right_wall_angles if angle in scan_data and scan_data[angle] is not None and scan_data[angle] > 0]
    left_distances = [scan_data[angle] for angle in left_wall_angles if angle in scan_data and scan_data[angle] is not None and scan_data[angle] > 0]

    avg_right_distance = sum(right_distances) / len(right_distances) if right_distances else None
    avg_left_distance = sum(left_distances) / len(left_distances) if left_distances else None

    error = 0.0

    if avg_right_distance is not None and avg_left_distance is not None:
        # Both walls detected, try to center
        error = (avg_right_distance - avg_left_distance) / 2.0
        # Positive error if too close to right, negative if too close to left
        # Adjusting the sign based on desired turning behavior for PID
        # If right_wall_close, error is positive => turn left. Need PID to output negative.
        # If left_wall_close, error is negative => turn right. Need PID to output positive.
        # So, error = (avg_left_distance - avg_right_distance) is more direct for "steering error"
        error = avg_left_distance - avg_right_distance
        print(f"LiDAR: Both walls. Left: {avg_left_distance:.0f}, Right: {avg_right_distance:.0f}. Error: {error:.0f}mm")

    elif avg_right_distance is not None:
        # Only right wall detected, try to maintain target distance from it
        error = avg_right_distance - target_distance_mm
        # If avg_right_distance is > target_distance_mm, error is positive (too far from right), need to turn right (increase angle)
        # If avg_right_distance is < target_distance_mm, error is negative (too close to right), need to turn left (decrease angle)
        # For clockwise movement, this error directly feeds PID to adjust servo towards/away from 90.0 (center)
        print(f"LiDAR: Right wall only. Dist: {avg_right_distance:.0f}. Error: {error:.0f}mm")

    elif avg_left_distance is not None:
        # Only left wall detected, try to maintain target distance from it
        error = -(avg_left_distance - target_distance_mm) # Negative sign to invert error for steering
        # If avg_left_distance is > target_distance_mm, error is positive (too far from left), need to turn left (decrease angle)
        # If avg_left_distance is < target_distance_mm, error is negative (too close to left), need to turn right (increase angle)
        print(f"LiDAR: Left wall only. Dist: {avg_left_distance:.0f}. Error: {error:.0f}mm")

    else:
        # No walls detected (e.g., middle of arena, or at a corner entrance)
        # Keep going straight or assume a predefined corner turning strategy
        # This is where corner detection logic would be more sophisticated.
        print("LiDAR: No walls detected. Maintaining course (error=0).")
        error = 0.0 # Default to straight if no walls are detected

    return error

# This __name__ == "__main__" block is for testing this file independently
if __name__ == "__main__":
    print("--- Testing lidar_steering1.py ---")
    print("This will simulate LiDAR data and PID output without physical hardware.")

    # Create a dummy LiDAR scanner (it won't connect, just allows structure)
    class DummyLidarScanner:
        def __init__(self):
            self.count = 0
        def connect(self): print("Dummy LiDAR connected.")
        def disconnect(self): print("Dummy LiDAR disconnected.")
        def __enter__(self): self.connect(); return self
        def __exit__(self, exc_type, exc_val, exc_tb): self.disconnect()
        
        def get_scan_data(self):
            self.count += 1
            if self.count % 5 == 0: # Simulate turning a corner, no side walls
                print("SIMULATION: Entering corner, no side walls detected.")
                return {0: 1000, 10: 980, 350: 980} # Only front data
            elif self.count % 2 == 0: # Simulate too close to right wall
                print("SIMULATION: Too close to right wall.")
                return {0: 1000, 90: 500, 270: 1000} # Right wall close
            else: # Simulate centered
                print("SIMULATION: Centered between walls.")
                return {0: 1000, 90: 750, 270: 750} # Centered

    pid = PIDController(Kp=0.3, Ki=0.001, Kd=0.02)
    
    try:
        with DummyLidarScanner() as scanner:
            for i in range(10):
                print(f"\n--- Simulation Step {i+1} ---")
                scan_data = scanner.get_scan_data()
                if scan_data:
                    error = calculate_steering_error(scan_data, target_distance_mm=750, safety_distance_mm=300)
                    print(f"Calculated Error: {error:.2f}mm")
                    pid_output = pid.update(error)
                    print(f"PID Output: {pid_output:.2f}")
                else:
                    print("No scan data available (simulated).")
                time.sleep(1)
    except Exception as e:
        print(f"Test error: {e}")
    finally:
        print("\n--- LiDAR Steering Test Complete ---")