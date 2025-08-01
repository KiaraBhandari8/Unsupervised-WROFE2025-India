import math
import ydlidar

# --- LiDAR Scanner Class (No changes needed) ---
class LidarScanner:
    def __init__(self, port='/dev/ttyUSB0', baudrate=230400):
        self.port = port
        self.baudrate = baudrate
        self.laser = None
        self.scan_data = {}

        self.MIN_ANGLE = -180.0
        self.MAX_ANGLE = 180.0
        self.MIN_RANGE = 0.02
        self.MAX_RANGE = 16.0

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def connect(self):
        try:
            ydlidar.os_init()
            self.laser = ydlidar.CYdLidar()

            self.laser.setlidaropt(ydlidar.LidarPropSerialPort, self.port)
            self.laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, self.baudrate)
            self.laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
            self.laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
            self.laser.setlidaropt(ydlidar.LidarPropSampleRate, 4)
            self.laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
            self.laser.setlidaropt(ydlidar.LidarPropMaxAngle, self.MAX_ANGLE)
            self.laser.setlidaropt(ydlidar.LidarPropMinAngle, self.MIN_ANGLE)
            self.laser.setlidaropt(ydlidar.LidarPropMaxRange, self.MAX_RANGE)
            self.laser.setlidaropt(ydlidar.LidarPropMinRange, self.MIN_RANGE)
            self.laser.setlidaropt(ydlidar.LidarPropIntenstiy, True)

            ret = self.laser.initialize()
            if not ret:
                raise IOError(f"LiDAR connection failed: {self.laser.DescribeError()}")

            ret = self.laser.turnOn()
            if not ret:
                raise IOError(f"Failed to turn on YDLIDAR: {self.laser.DescribeError()}")

            print(f"LiDAR: Connected to {self.port} at {self.baudrate} baud.")
        except Exception as e:
            print(f"LiDAR ERROR: Could not connect to LiDAR: {e}")
            raise IOError(f"LiDAR connection failed: {e}")

    def disconnect(self):
        if self.laser:
            print("LiDAR: Disconnecting...")
            self.laser.turnOff()
            self.laser.disconnecting()
            self.laser = None
            print("LiDAR: Disconnected.")

    def get_scan_data(self):
        if not self.laser:
            return None

        self.scan_data = {}
        scan = ydlidar.LaserScan()

        try:
            r = self.laser.doProcessSimple(scan)
            if r:
                for p in scan.points:
                    if self.MIN_RANGE <= p.range <= self.MAX_RANGE:
                        angle_degrees = round(math.degrees(p.angle))
                        distance_mm = p.range * 1000
                        self.scan_data[angle_degrees] = distance_mm
                return self.scan_data
            else:
                return None
        except Exception as e:
            print(f"LiDAR DATA ERROR: {e}")
            return None

def calculate_potential_field_steering(scan_data):
    """
    Calculates steering angle to stay between left and right walls.
    Repulsive forces are only generated from the side walls.

    Args:
        scan_data (dict): Dictionary of angle:distance_mm from LiDAR.

    Returns:
        float: The desired steering angle. Positive for left, negative for right.
    """
    # --- Parameters to Tune ---
    ATTRACTIVE_STRENGTH = 10000.0
    REPULSIVE_STRENGTH = 200000.0
    WALL_RADIUS_MM = 600.0

    if not scan_data:
        return 0.0

    # 1. Attractive Force (Goal) - Pulls the robot straight forward
    attractive_vx = ATTRACTIVE_STRENGTH
    attractive_vy = 0.0

    # 2. Repulsive Forces (Walls)
    total_repulsive_vx = 0.0
    total_repulsive_vy = 0.0

    for angle_deg, distance_mm in scan_data.items():
        # *** MODIFICATION: Define angle ranges for the walls ***
        # Check if the detected point is part of the left or right wall.
        is_left_wall = -90 <= angle_deg <= -30
        is_right_wall = 30 <= angle_deg <= 90
        
        # Only calculate repulsive forces for points on the side walls
        if (is_left_wall or is_right_wall) and (0 < distance_mm < WALL_RADIUS_MM):
            angle_rad = math.radians(angle_deg)
            
            # The force is stronger for closer walls
            repulsive_force = REPULSIVE_STRENGTH * (1.0 / distance_mm - 1.0 / WALL_RADIUS_MM)
            
            # Add the repulsive vector (points away from the wall)
            total_repulsive_vx -= repulsive_force * math.cos(angle_rad)
            total_repulsive_vy -= repulsive_force * math.sin(angle_rad)

    # 3. Combine Forces to get the final vector
    final_vx = attractive_vx + total_repulsive_vx
    final_vy = attractive_vy + total_repulsive_vy

    # 4. Calculate the steering angle from the final vector
    steering_angle = math.degrees(math.atan2(final_vy, final_vx))
    
    return steering_angle