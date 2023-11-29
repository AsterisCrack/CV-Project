from gpiozero import Servo
from time import sleep

servo = Servo(22)

while True:
   #Ask for angle between 0 and 180
    angle = float(input('angle: '))
    servo.value = angle/180
   # sleep(0.1)
   # servo.value = None
   # sleep(0.1)

     