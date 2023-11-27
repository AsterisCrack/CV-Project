#Control servomotors with raspberry pi
import RPi.GPIO as GPIO
from simple_pid import PID
from tracker import Tracker
import time
import math

class ServoController:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pin, 50)
        self.pwm.start(0)
        self.angle = 0
        self.angle_min = 0
        self.angle_max = 180
        self.duty_min = 2.5
        self.duty_max = 12.5
        self.duty_span = self.duty_max - self.duty_min
        self.angle_span = self.angle_max - self.angle_min
        self.angle_to_duty = lambda angle: self.duty_min + (angle-self.angle_min) * (self.duty_span/self.angle_span)

    def set_angle(self, angle):
        self.angle = angle
        duty = self.angle_to_duty(angle)
        self.pwm.ChangeDutyCycle(duty)

    def get_angle(self):
        return self.angle

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()

    def __del__(self):
        self.cleanup()


if __name__ == '__main__':
    servoPitch = ServoController(22)
    servoYaw = ServoController(27)#or 17 
    PID_pitch = PID(0.01, 0.5, 0.05, output_limits=(0, 100), setpoint=0)
    PID_yaw = PID(1, 0.1, 0.05, output_limits=(0, 100), setpoint=0)
    def move_robot(center):
    #Moves the robot to point at the center of the object
    # center is a tuple (x, y)
    # 
        error_x = center[0] - 320
        error_y = center[1] - 240
        angle_x = PID_pitch(error_x)
        angle_y = PID_yaw(error_y)
        
       # Invert angle
        angle_x = -angle_x + 100 
        servoPitch.set_angle(angle_x)
        servoYaw.set_angle(angle_y)
        print("x: ", error_x, "y: ", error_y)
        print("angle_x: ", angle_x, "angle_y: ", angle_y)

    tracker = Tracker(on_new_frame_function=move_robot) 

    tracker.track()
    tracker.close()
    servoPitch.cleanup()
    servoYaw.cleanup()
    GPIO.cleanup()



    