import os
import ydlidar
import time

# --- Correct Configuration for YDLIDAR T-mini Plus ---

# Initialize the system signal handler (for Ctrl+C)
ydlidar.os_init()

# Create a YDLidar object
laser = ydlidar.CYdLidar()

# --- Critical Settings Found From Example ---
port_name = "/dev/ttyUSB0" 
laser.setlidaropt(ydlidar.LidarPropSerialPort, port_name)

# ** THE KEY FIX: Baud rate for T-mini Plus is 230400 **
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)

laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
laser.setlidaropt(ydlidar.LidarPropSampleRate, 4) # Correct sample rate is 4K
laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)

# Set the correct angle and range for the T-mini Plus
laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0) # Max range is 16 meters
laser.setlidaropt(ydlidar.LidarPropMinRange, 0.02) # Min range is 0.02 meters

# This model supports intensity data
laser.setlidaropt(ydlidar.LidarPropIntenstiy, True)

# --- Initialization and Scan ---
print("Initializing YDLIDAR with correct settings...")
ret = laser.initialize()

if ret:
    print("Initialization successful.")
    ret = laser.turnOn()
    if not ret:
        print(f"Failed to turn on YDLIDAR: {laser.DescribeError()}")
        laser.disconnecting()
        exit()
    
    print("\n--- LiDAR is scanning ---")
    scan = ydlidar.LaserScan()
    
    try:
        while ret and ydlidar.os_isOk():
            r = laser.doProcessSimple(scan)
            if r:
                # A full 360° scan has been received
                num_points = len(scan.points)
                if num_points > 0:
                    # Print the total number of points in the scan
                    print(f"Scan received with {num_points} points.")
                    
                    # You can now process all the points in the scan.
                    # For example, let's look at the point directly in front (0 degrees)
                    front_point = min(scan.points, key=lambda p: abs(p.angle))
                    angle_deg = front_point.angle * 180.0 / 3.14159
                    distance_m = front_point.range
                    
                    print(f"  > Point closest to 0°: Angle: {angle_deg:.2f}°, Distance: {distance_m:.2f} m")
                    print("-" * 20)
                
            else:
                print("Failed to get LiDAR data. Retrying...")
                time.sleep(0.05)
                
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        print("Shutting down LiDAR...")
        laser.turnOff()
        laser.disconnecting()
        print("YDLIDAR disconnected.")
else:
    print(f"Failed to initialize YDLIDAR: {laser.DescribeError()}")