import os
import ydlidar
import time
import math
import matplotlib
# IMPORTANT: Use 'Agg' backend for headless environments
# This MUST be set BEFORE importing pyplot
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from datetime import datetime

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
# Store these values in variables as we can't 'getlidaropt' directly
MIN_ANGLE = -180.0
MAX_ANGLE = 180.0
MIN_RANGE = 0.02 # Explicitly use this value for filtering
MAX_RANGE = 12

laser.setlidaropt(ydlidar.LidarPropMaxAngle, MAX_ANGLE)
laser.setlidaropt(ydlidar.LidarPropMinAngle, MIN_ANGLE)
laser.setlidaropt(ydlidar.LidarPropMaxRange, MAX_RANGE)
laser.setlidaropt(ydlidar.LidarPropMinRange, MIN_RANGE)

# This model supports intensity data
laser.setlidaropt(ydlidar.LidarPropIntenstiy, True)

# --- Matplotlib Setup for Saving Figures ---
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='polar')

# Set static plot properties once using the stored variables
ax.set_thetamin(MIN_ANGLE)
ax.set_thetamax(MAX_ANGLE)
ax.set_rmax(MAX_RANGE) # Max range of the LiDAR
ax.set_title("YDLIDAR 360° Scan Map", va='bottom')
ax.set_xlabel("Angle (degrees)")
ax.set_ylabel("Distance (meters)")
ax.grid(True)

# Create a directory to save the maps if it doesn't exist
output_dir = "lidar_maps"
os.makedirs(output_dir, exist_ok=True)
# Define the fixed filename for the latest map
output_filename = os.path.join(output_dir, "latest_lidar_map.png")
print(f"Latest map will be saved as: {output_filename}")

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

    last_save_time = time.time()
    save_interval_sec = 2 # Save image every 5 seconds

    try:
        while ret and ydlidar.os_isOk():
            r = laser.doProcessSimple(scan)
            current_time = time.time()

            if r:
                num_points = len(scan.points)
                if num_points > 0:
                    angles = []
                    ranges = []
                    for p in scan.points:
                        # Use the MIN_RANGE variable directly for filtering
                        if p.range >= MIN_RANGE and p.range <= MAX_RANGE: # Ensure it's within valid range
                            angles.append(p.angle)
                            ranges.append(p.range)

                            # --- NEW: Print angle in degrees and range in cm ---
                            angle_degrees = math.degrees(p.angle) # Convert radians to degrees
                            distance_cm = p.range * 100 # Convert meters to centimeters
                            print(f"Angle: {angle_degrees:.2f} degrees, Distance: {distance_cm:.2f} cm")

                    # Check if it's time to save an image
                    if current_time - last_save_time >= save_interval_sec:
                        if angles and ranges:
                            # Clear previous plot data and plot new data
                            ax.clear() # Clear the previous plot to avoid overlay
                            ax.plot(angles, ranges, 'o', markersize=2) # Plot the new points

                            # Re-apply static plot settings after clearing
                            ax.set_thetamin(MIN_ANGLE)
                            ax.set_thetamax(MAX_ANGLE)
                            ax.set_rmax(MAX_RANGE)
                            ax.set_title("YDLIDAR 360° Scan Map", va='bottom')
                            ax.set_xlabel("Angle (degrees)")
                            ax.set_ylabel("Distance (meters)")
                            ax.grid(True)

                            try:
                                plt.savefig(output_filename)
                                print(f"  > Saved map to: {output_filename} at {datetime.now().strftime('%H:%M:%S')}")
                                last_save_time = current_time # Reset timer
                            except Exception as e:
                                print(f"  ERROR: Could not save figure to {output_filename}: {e}")
                        else:
                            print("  No valid points to plot in this scan, skipping image save.")
                            # Optionally, reset last_save_time here if you only want to count time from a successful plot
                            # last_save_time = current_time
                else:
                    print("Scan received, but no points found.")

            else:
                print("Failed to get LiDAR data. Retrying...")
            time.sleep(0.01) # Short sleep to prevent busy-waiting, adjust as needed

    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        print("Shutting down LiDAR...")
        laser.turnOff()
        laser.disconnecting()
        print("YDLIDAR disconnected.")
        # Ensure the figure is closed to free resources
        plt.close(fig)
        print("Matplotlib figure closed.")
else:
    print(f"Failed to initialize YDLIDAR: {laser.DescribeError()}")