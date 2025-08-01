import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
import robot_motion

# TRIG and ECHO pin mappings
# The 'dist' key isn't strictly necessary in this dictionary
# if you're returning the values directly from the function.
sensors = [
    {'trig': 5, 'echo': 6, 'placement' : 'left'},     # Sensor Left
    {'trig': 13, 'echo': 19, 'placement' : 'forward'},   # Sensor Forward
    {'trig': 26, 'echo': 21, 'placement' : 'right'},   # Sensor Right
]

# Set up all pins
for sensor in sensors:
    GPIO.setup(sensor['trig'], GPIO.OUT)
    GPIO.setup(sensor['echo'], GPIO.IN)
    GPIO.output(sensor['trig'], False)

time.sleep(1)  # Allow sensors to settle

def measure_distance(trig, echo):
  
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    start_time = time.time()
    stop_time = time.time()

    # Wait for the ECHO pin to go HIGH
    timeout_start = time.time()
    while GPIO.input(echo) == 0:
        start_time = time.time()
        if (time.time() - timeout_start) > 1: # Timeout after 1 second
            return -1 # Indicate an error or no signal

    # Wait for the ECHO pin to go LOW
    timeout_start = time.time()
    while GPIO.input(echo) == 1:
        stop_time = time.time()
        if (time.time() - timeout_start) > 1: # Timeout after 1 second
            return -1 # Indicate an error or no signal

    elapsed_time = stop_time - start_time
    # Speed of sound in air is approximately 34300 cm/s
    distance = (elapsed_time * 34300) / 2
    return round(distance)

def get_sensor_distances():

    left_dist = -1
    forward_dist = -1
    right_dist = -1

    for sensor in sensors:
        dist = measure_distance(sensor['trig'], sensor['echo'])
        if sensor['placement'] == 'left':
            left_dist = dist
        elif sensor['placement'] == 'forward':
            forward_dist = dist
        elif sensor['placement'] == 'right':
            right_dist = dist
    return left_dist, right_dist, forward_dist

def get_state_angle():
    """
    This func will return 
    1. State --> 'Between Lines', 'Left Corner', 'Right Corner'
    2. Angle ---> If State is between lines the compute angle to stay in the center, else None
    """
    state = None
    angle = None
    maxangle = 45
    ld, rd, fd = get_sensor_distances()
    if(ld < 40 and rd < 40):
        state = 'BL'
        angle = round((ld-rd)*maxangle/75)
    elif(ld>40 and rd<40):
        state = 'LC'
    elif(ld<40 and rd > 40):
        state = 'RC'
    elif(ld>40 and rd>40):
        state = None
        angle = None
    elif fd < 20:
        state = 'ST'
    
    return state, angle, ld, rd, fd
# --- Main loop to demonstrate usage ---

if __name__ == "__main__":

    try:
        while True:
            robot_motion.robot_forward()
            state, steering_angle, ld, rd, fd = get_state_angle()
            print(f"state : {state}, angle: {steering_angle}, left_distance: {ld}, right_distance: {rd}, forward_distance: {fd}")
            if state == 'BL':
                final_steering_angle = 75 + steering_angle
                robot_motion.adjust_servo_angle(final_steering_angle)
            if state == 'ST':
                robot_motion.robot_stop()
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        GPIO.cleanup()