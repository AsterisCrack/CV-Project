"""
This program will be the main program that runs on the raspberry pi.

It will consist on two parts: 
    1. the control of the access, which will be a sequence of specific shapes of given colors. This sequence is called the password.
    2. The tracking of the object on the camera, 
"""


from deteccion_secuencia import Camera, Password_program_constants # This is the class that detects the password
from robot_controller import Robot_Controller # This is the class that controls the robot


import cv2
import numpy as np
from time import time

#packages used for the robot control
import RPi.GPIO as GPIO
from gpiozero import Servo  
from gpiozero.pins.pigpio import PiGPIOFactory
from simple_pid import PID
from tracker import Tracker
import RobotControl.remove_servo_jitter as remove_jitter
import RobotControl.led_controller as led_controller



if __name__ == "__main__":
    # The program starts by creating the camera object in order to detect the password.
    tipo = "webcam" # "pi"
    cam = Camera(type= tipo, constants=Password_program_constants())
    frame = cam.get_frame()
    password_loop = True

    while password_loop:
        cv2.imshow("frame", frame)
        password_loop = cam.process_video()

    cam.end_program()
    # delete the camera object to free up memory
    del cam
    
    # The password has been detected, so the tracker object is created.
    remove_jitter.remove_jitter()
    PinFactory = PiGPIOFactory()

    INVERT_X = True
    INVERT_Y = True
    PIN_X = 22
    PIN_Y = 27
    COLOR = "blue"
    BRIGHTNESS = 20

    servo_x= Servo(PIN_X, pin_factory=PinFactory, min_pulse_width=0.0005, max_pulse_width=0.0025)
    servo_y = Servo(PIN_Y, pin_factory=PinFactory, min_pulse_width=0.0005, max_pulse_width=0.0025)
    servo_x.value = 0
    servo_y.value = 0
    PID_x = PID(0.1, 1, 0.005, output_limits=(-1, 1), setpoint=0, starting_output=0)
    PID_y = PID(0.075, 0.75, 0.005, output_limits=(-0.5, 0.5), setpoint=0, starting_output=0)


    #global start_control
    
    robot = Robot_Controller(INVERT_X=INVERT_X, INVERT_Y=INVERT_Y, 
                             servo_x=servo_x, servo_y=servo_y, 
                             pid_x=PID_x, pid_y=PID_y, 
                             COLOR=COLOR, BRIGHTNESS=BRIGHTNESS,
                             leds=led_controller)

    robot.start(Tracker)

    