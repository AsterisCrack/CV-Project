#Control servomotors with raspberry pi
import RPi.GPIO as GPIO
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
        time.sleep(0.3)

    def get_angle(self):
        return self.angle

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()

    def __del__(self):
        self.cleanup()

if __name__ == '__main__':
    servo = ServoController(18)
    try:
        while True:
            angle = float(input('angle: '))
            servo.set_angle(angle)
    except KeyboardInterrupt:
        servo.cleanup()