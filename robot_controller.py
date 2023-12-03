#Control servomotors with raspberry pi
import RPi.GPIO as GPIO
from gpiozero import Servo  
from gpiozero.pins.pigpio import PiGPIOFactory
from simple_pid import PID
from tracker import Tracker
import RobotControl.remove_servo_jitter as remove_jitter
import time
import math

class Robot_Controller:

    def __init__(self, 
                 INVERT_X: bool = True, INVERT_Y: bool = True, 
                 servo_x = None, servo_y = None, 
                 pid_x = None, pid_y = None,                 
                 COLOR: str = "blue"
                 ):
        
        self.INVERT_X = INVERT_X
        self.INVERT_Y = INVERT_Y

    
        self.COLOR = COLOR
        self.obj_detected = False
        self.start_control = False
        
        self.servo_x= servo_x #Servo(self.PIN_X, pin_factory=PinFactory, min_pulse_width=min_pulse_width, max_pulse_width=max_pulse_width)
        self.servo_y = servo_y #Servo(self.PIN_Y, pin_factory=PinFactory, min_pulse_width=min_pulse_width, max_pulse_width=max_pulse_width)
        self.servo_x.value = 0
        self.servo_y.value = 0
        self.PID_x = pid_x #PID(0.1, 1, 0.005, output_limits=(-1, 1), setpoint=0, starting_output=0)
        self.PID_y = pid_y #PID(0.075, 0.75, 0.005, output_limits=(-0.5, 0.5), setpoint=0, starting_output=0)


    def move_robot(self, center):
        #Moves the robot to point at the center of the object
        # center is a tuple (x, y)
        
        if center is None:
            self.servo_x.value = 0
            self.servo_y.value = 0
            return
        
        error_x = (center[0] - 320) / 320
        error_y = (center[1] - 240) / 240
        angle_x = self.PID_x(error_x)
        angle_y = self.PID_y(error_y)

        #global start_control
        if not self.start_control:
            if angle_x != -1 and angle_x != 1 and angle_y != -1 and angle_y != 1:
                self.start_control = True
            return
        
       # Invert angle
        if self.INVERT_X:
            angle_x = -angle_x
        if self.INVERT_Y:
            angle_y = -angle_y

        self.servo_x.value = angle_x
        self.servo_y.value = angle_y

    def setup(self, tracker_obj):
        self.tracker = tracker_obj(on_new_frame_function=self.move_robot, color=self.COLOR)

    def start(self, tracker_obj):
        print("Tracker startup")
        self.setup(tracker_obj)
        input("Press enter to start tracking")
        self.tracker.track()
        self.close()

    def close(self):
        self.tracker.close()
        self.servo_x.value = 0
        self.servo_y.value = 0
        self.servo_x.close()
        self.servo_y.close()

#global obj_detected 
#obj_detected = False

if __name__ == '__main__':

    #Remove jitter from servo 
    remove_jitter.remove_jitter()
    PinFactory = PiGPIOFactory()

    INVERT_X = True
    INVERT_Y = True
    PIN_X = 22
    PIN_Y = 27
    COLOR = "red"

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
                             COLOR=COLOR)

    robot.start(Tracker)


    