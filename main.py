"""
This program will be the main program that runs on the raspberry pi.

It will consist on two parts: 
    1. the control of the access, which will be a sequence of specific shapes of given colors. This sequence is called the password.
    2. The tracking of the object on the camera, 
"""

from tracker import Tracker # this will be created when the password is detected.
from shape_color_detection import Camera, Password_program_constants

import cv2
import numpy as np
from time import time


if __name__ == "__main__":
    # The program starts by creating the camera object in order to detect the password.
    tipo = "webcam" # "pi"
    cam = Camera(type= tipo)
    frame = cam.get_frame()
    password_loop = True

    while password_loop:
        cv2.imshow("frame", frame)
        password_loop = cam.process_video()

    cam.end_program()
    # delete the camera object to free up memory
    del cam
    
    # The password has been detected, so the tracker object is created.
    tracker = Tracker() # use the default values
    tracker.track()
    tracker.close()

    