import time
from gpiozero import DistanceSensor, LED, Motor

sensor = DistanceSensor(echo=18, trigger=17, max_distance=3.0) 

green_led = LED(22)  
red_led = LED(27)    

left_motor = Motor(forward=23, backward=24)
right_motor = Motor(forward=5, backward=6)

OBSTACLE_THRESHOLD_METERS = 0.914 

def move_forward():
    red_led.off()
    green_led.on()
    left_motor.forward()
    right_motor.forward()

def execute_avoidance_maneuver():
    left_motor.stop()
    right_motor.stop()
    
    green_led.off()
    red_led.on()
    print("Obstacle detected within 3ft. Executing avoidance maneuver...")
    
    left_motor.backward()
    right_motor.backward()
    time.sleep(1.0)
    
    left_motor.forward()
    right_motor.backward()
    time.sleep(1.5)

try:
    print("Battery power applied. Initializing systems...")
    time.sleep(2)
    
    while True:
        current_distance = sensor.distance
        
        if current_distance <= OBSTACLE_THRESHOLD_METERS:
            execute_avoidance_maneuver()
        else:
            move_forward()
            
        time.sleep(0.05)

except KeyboardInterrupt:
    left_motor.stop()
    right_motor.stop()
    green_led.off()
    red_led.off()
    print("\nPower disconnected. Systems off.")