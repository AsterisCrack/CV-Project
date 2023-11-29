#Control servomotors with raspberry pi
import RPi.GPIO as GPIO
from gpiozero import Servo  
from gpiozero.pins.pigpio import PiGPIOFactory
from simple_pid import PID
from tracker import Tracker
import RobotControl.remove_servo_jitter as remove_jitter
import RobotControl.led_controller as led_controller
import time
import math

#Remove jitter from servo 
remove_jitter.remove_jitter()
PinFactory = PiGPIOFactory()

INVERT_X = True
INVERT_Y = True
PIN_X = 22
PIN_Y = 27
COLOR = "blue"
BRIGHTNESS = 20
global obj_detected 
obj_detected = False

if __name__ == '__main__':
   
    servo_x= Servo(PIN_X, pin_factory=PinFactory, min_pulse_width=0.0005, max_pulse_width=0.0025)
    servo_y = Servo(PIN_Y, pin_factory=PinFactory, min_pulse_width=0.0005, max_pulse_width=0.0025)
    servo_x.value = 0
    servo_y.value = 0
    PID_x = PID(0.1, 1, 0.005, output_limits=(-1, 1), setpoint=0, starting_output=0)
    PID_y = PID(0.075, 0.75, 0.005, output_limits=(-0.5, 0.5), setpoint=0, starting_output=0)
    global start_control
    start_control = False
    def move_robot(center):
        #Moves the robot to point at the center of the object
        # center is a tuple (x, y)
        
        if center is None:
            servo_x.value = 0
            servo_y.value = 0
            led_controller.change_effect("breathe")
            return
        
        error_x = (center[0] - 320) / 320
        error_y = (center[1] - 240) / 240
        angle_x = PID_x(error_x)
        angle_y = PID_y(error_y)
        
        global start_control
        if not start_control:
            if angle_x != -1 and angle_x != 1 and angle_y != -1 and angle_y != 1:
                start_control = True
            return
        
       # Invert angle
        if INVERT_X:
            angle_x = -angle_x
        if INVERT_Y:
            angle_y = -angle_y

        servo_x.value = angle_x
        servo_y.value = angle_y
        led_controller.change_effect("static_color")

   #Set up LEDs
    led_controller.change_brightness(BRIGHTNESS)
    led_controller.change_color(COLOR)
    led_controller.change_effect("breathe")
    tracker = Tracker(on_new_frame_function=move_robot, color=COLOR)
    
   #Wait for key when ready
    input("Press enter to start tracking")

    tracker.track()
    
   #Closing sequence 
    tracker.close()

    servo_x.value = 0
    servo_y.value = 0

    servo_x.close()
    servo_y.close()

    