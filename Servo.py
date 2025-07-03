from adafruit_servokit import ServoKit
from time import sleep

kit = ServoKit(channels=16)

for i in range(60,120,5):
    kit.servo[1].angle = i
    print(f"Servo Angle : {i}")
    sleep(0.2)
    
sleep(3)
for i in range(120,60,-5):
    kit.servo[1].angle = i
    print(f"Servo Angle : {i}")
    sleep(0.2)