#Control servomotors with raspberry pi
import RPi.GPIO as GPIO
from gpiozero import Servo  
from gpiozero.pins.pigpio import PiGPIOFactory
from simple_pid import PID
from tracker import Tracker
import RobotControl.remove_servo_jitter as remove_jitter
# import RobotControl.led_controller as led_controller
import time
import math

class Robot_Controller:

    def __init__(self, 
                 INVERT_X: bool = True, INVERT_Y: bool = True, 
                 servo_x = None, servo_y = None, 
                 pid_x = None, pid_y = None,                 
                 
                 COLOR: str = "blue", BRIGHTNESS: int = 20, 
                 
                 leds = None
                 ):
        
        self.INVERT_X = INVERT_X
        self.INVERT_Y = INVERT_Y

    
        self.COLOR = COLOR
        self.BRIGHTNESS = BRIGHTNESS
        self.obj_detected = False
        self.start_control = False
        
        self.servo_x= servo_x #Servo(self.PIN_X, pin_factory=PinFactory, min_pulse_width=min_pulse_width, max_pulse_width=max_pulse_width)
        self.servo_y = servo_y #Servo(self.PIN_Y, pin_factory=PinFactory, min_pulse_width=min_pulse_width, max_pulse_width=max_pulse_width)
        self.servo_x.value = 0
        self.servo_y.value = 0
        self.PID_x = pid_x #PID(0.1, 1, 0.005, output_limits=(-1, 1), setpoint=0, starting_output=0)
        self.PID_y = pid_y #PID(0.075, 0.75, 0.005, output_limits=(-0.5, 0.5), setpoint=0, starting_output=0)

        self.leds = leds #led_controller 


    def move_robot(self, center):
        #Moves the robot to point at the center of the object
        # center is a tuple (x, y)
        
        if center is None:
            self.servo_x.value = 0
            self.servo_y.value = 0
            self.leds.change_effect("breathe")
            return
        
        error_x = (center[0] - 320) / 320
        error_y = (center[1] - 240) / 240
        angle_x = self.PID_x(error_x)
        angle_y = self.PID_y(error_y)
        
        #global start_control
        if not self.start_control:
            if angle_x != -1 and angle_x != 1 and angle_y != -1 and angle_y != 1: #????
                self.start_control = True
            return
        
       # Invert angle
        if self.INVERT_X:
            angle_x = -angle_x
        if self.INVERT_Y:
            angle_y = -angle_y

        self.servo_x.value = angle_x
        self.servo_y.value = angle_y
        self.leds.change_effect("static_color")

    def setup_leds_tracker(self, tracker_obj):
        self.leds.change_brightness(self.BRIGHTNESS)
        self.leds.change_color(self.COLOR)
        self.leds.change_effect("breathe")
        self.tracker = tracker_obj(on_new_frame_function=self.move_robot, color=self.COLOR)

    def start(self, tracker_obj):
        self.setup_leds_tracker(tracker_obj)
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
<<<<<<< HEAD
    try:
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
               # led_controller.change_effect("breathe")
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
           # led_controller.change_effect("static_color")

    #Set up LEDs
       # led_controller.change_brightness(BRIGHTNESS)
       # led_controller.change_color(COLOR)
       # led_controller.change_effect("static_color")
        tracker = Tracker(on_new_frame_function=move_robot, color=COLOR)
=======

    #Remove jitter from servo 
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

    """
    start_control = False
    def move_robot(center):
        #Moves the robot to point at the center of the object
        # center is a tuple (x, y)
>>>>>>> 89b061459382f29561ac99bf403fe86c727facbf
        
        #Closing sequence 
        def close_program():
            print("closing all systems")
            tracker.close()

            servo_x.value = 0
            servo_y.value = 0

<<<<<<< HEAD
            servo_x.close()
            servo_y.close()
           # led_controller.stop_all()
           # GPIO.cleanup()
            exit()

    #Wait for key when ready
        input("Press enter to start tracking")
=======
        servo_x.value = angle_x
        servo_y.value = angle_y
        led_controller.change_effect("static_color")"""
"""
   #Set up LEDs
    led_controller.change_brightness(BRIGHTNESS)
    led_controller.change_color(COLOR)
    led_controller.change_effect("breathe")
    tracker = Tracker(on_new_frame_function=move_robot, color=COLOR)
    
   #Wait for key when ready
    input("Press enter to start tracking")
>>>>>>> 89b061459382f29561ac99bf403fe86c727facbf

        tracker.track()
        
        close_program()

    except KeyboardInterrupt:
        close_program()
        

<<<<<<< HEAD
    except Exception as e:
        print(e)
        close_program()
        

=======
    servo_x.close()
    servo_y.close()
"""
>>>>>>> 89b061459382f29561ac99bf403fe86c727facbf
    