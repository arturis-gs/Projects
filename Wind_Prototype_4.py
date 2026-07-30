import time
from gpiozero import DistanceSensor, Motor, AngularServo
from mpu6050 import mpu6050

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def compute(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

class Prototype_4:
    def __init__(self):
        self.throttle_pid = PIDController(1.5, 0.1, 0.5, 50.0)
        self.steering_pid = PIDController(2.0, 0.05, 1.0, 50.0)
        self.aero_pid = PIDController(3.0, 0.2, 0.8, 0.0)
        self.last_time = time.time()
        
        self.front_sensor = DistanceSensor(echo=18, trigger=17, max_distance=2.0)
        self.left_sensor = DistanceSensor(echo=24, trigger=23, max_distance=1.0)
        self.imu = mpu6050(0x68)
        
        self.main_motor = Motor(forward=5, backward=6)
        self.steering_servo = AngularServo(12, min_angle=-90, max_angle=90)
        self.flap_servo = AngularServo(13, min_angle=-45, max_angle=45)

    def read_front_distance(self):
        return self.front_sensor.distance * 100

    def read_left_distance(self):
        return self.left_sensor.distance * 100

    def read_pitch_angle(self):
        accel_data = self.imu.get_accel_data()
        return accel_data['x']

    def set_throttle(self, value):
        clamped_value = max(-1.0, min(1.0, value / 100.0))
        if clamped_value > 0:
            self.main_motor.forward(clamped_value)
        elif clamped_value < 0:
            self.main_motor.backward(abs(clamped_value))
        else:
            self.main_motor.stop()

    def set_steering(self, angle):
        clamped_angle = max(-90.0, min(90.0, angle))
        self.steering_servo.angle = clamped_angle

    def set_aero_flaps(self, angle):
        clamped_angle = max(-45.0, min(45.0, angle))
        self.flap_servo.angle = clamped_angle

    def stabilize(self):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            dt = 0.01

        front_dist = self.read_front_distance()
        throttle_adjust = self.throttle_pid.compute(front_dist, dt)
        self.set_throttle(throttle_adjust)

        left_dist = self.read_left_distance()
        steering_adjust = self.steering_pid.compute(left_dist, dt)
        self.set_steering(steering_adjust)

        pitch = self.read_pitch_angle()
        flap_adjust = self.aero_pid.compute(pitch, dt)
        self.set_aero_flaps(flap_adjust)

        self.last_time = current_time

car = Prototype_4()

try:
    while True:
        car.stabilize()
        time.sleep(0.01)
except KeyboardInterrupt:
    car.set_throttle(0)
    car.set_steering(0)
    car.set_aero_flaps(0)