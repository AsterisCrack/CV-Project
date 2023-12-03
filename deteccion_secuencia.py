from typing import Any
import cv2
import numpy as np
from time import time
#from picamera2 import PiCamera

class Password_program_constants:
    """
    This class contains the constants used throughout the program.
    """
    def __init__(self) -> None:
        # define the lower and upper bounds of the colors in the HSV color space
        self.RED_LOWER_BOUND_1 = np.array([0, 100, 20])
        self.RED_UPPER_BOUND_1 = np.array([10, 255, 255])
        self.RED_LOWER_BOUND_2 = np.array([160,100,20])
        self.RED_UPPER_BOUND_2 = np.array([179,255,255])
        """
        Because red is the color at the beginning/end of the color circle in the HSV
        color space, we may want to define two ranges for red, one for the beginning
        and one for the end of the color circle, so that we can detect colors at either end
        of the color spectrum.
        """

        self.BLUE_LOWER_BOUND = np.array([100, 100, 20])
        self.BLUE_UPPER_BOUND = np.array([120, 255, 255])

        self.GREEN_LOWER_BOUND = np.array([45, 100, 20])
        self.GREEN_UPPER_BOUND = np.array([76, 255, 255])

        self.YELLOW_LOWER_BOUND = np.array([20, 100, 20])
        self.YELLOW_UPPER_BOUND = np.array([40, 255, 255])


        self.SQUARE_ASPECT_RATIO_TOLERANCE = 0.05 # how far off the aspect ratio of a cuadrilateral
        # can be from 1.0 (a square) and still be considered a square. 

        self.TOLERANCE_DEFECT_POLYGON_CONTOUR = 0.1 # how far off the approximation of a polygon

        self.TIME_TO_DETECT = 5 # the time in seconds the program has to detect the next shape once it has detected a shape of the pattern



class Camera:
    def __init__(self, type:str = "webcam") -> object:
        """
        Intialiser for the camera object. This object will handle the operation of the camera throughout the program.


        Args:
            type (str): type of camera to be used. Either 'pi' or 'webcam'

        Returns:
            object: an object of the camera class
        """
        self.constants = Password_program_constants()
        self.type = type
        self.frameWidth = 1280
        self.frameHeight = 720
        self.color_sequence_index = 0
        self.last_object_detected_time = time()


        cam = None
        """
        if type == "pi":
            cam = Picamera2()
            cam.preview_configuration.main.size=(1280, 720)
            cam.preview_configuration.main.format="RGB888"
            cam.preview_configuration.align()
            cam.configure("preview")
            cam.start()"""

        if type == "webcam":
            cam = cv2.VideoCapture(0)
            cam.set(3, self.frameWidth)
            cam.set(4, self.frameHeight)

        self.cam_obj = cam

    def get_frame(self)-> np.ndarray:
        """
        Gets a frame from the camera and returns it as an image.

        Returns:
            np.ndarray: an image from the camera (BGR format)
        """
        if self.type == "pi":
            frame = self.cam_obj.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR) # this is if the camera reads in RGB format
            
        if self.type == "webcam":
            _, frame = self.cam_obj.read()

        return frame
    
    def end_program(self) -> None:
        """
        Ends the program by closing all the windows and stopping the camera.
        """
        cv2.destroyAllWindows()
        
        self.stop_camera()

    def stop_camera(self) -> None:
        """
        Stops the camera from capturing images.
        """
        if self.type == "pi":
            self.cam_obj.stop()
        
        if self.type == "webcam":
            self.cam_obj.release()

    def next_color(self) -> None:
        """
        Function that updates the color index to the next color in the sequence.

        Called once the program has detected the pertinent shape of the current color.
        """
        self.color_sequence_index += 1
        """
        if self.color_sequence_index > 3:
            print("Sequence complete!")
            self.end_program()
            exit()"""

    def reset_color_sequence(self) -> None:
        """
        Resets the color sequence index to 0. 

        Called when the program timeouts while searching for the next shape in the sequence. 
        """
        self.color_sequence_index = 0
        
    def process_red_video(self) -> bool:
        """
        Function that tries to detect a red rectangle in the video feed. We define a 
        rectangle as a cuadrilateral with an aspect ratio smaller than 1-SQUARE_ASPECT_RATIO_TOLERANCE
        or larger than 1+SQUARE_ASPECT_RATIO_TOLERANCE.
        
        Returns:
            bool: True if a red rectangle is detected, False otherwise.
        """

        # get the frame from the camera
        frame = self.get_frame()
        cv2.imshow("frame", frame)

        # convert the frame to HSV color space
        frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # create a mask for the red color
        mask1 = cv2.inRange(frame_hsv, self.constants.RED_LOWER_BOUND_1, self.constants.RED_UPPER_BOUND_1)
        mask2 = cv2.inRange(frame_hsv, self.constants.RED_LOWER_BOUND_2, self.constants.RED_UPPER_BOUND_2)
        mask = cv2.bitwise_or(mask1, mask2)

        # apply the mask to the frame
        red_frame = cv2.bitwise_and(frame, frame, mask=mask)

        # convert the masked frame to grayscale
        red_frame = cv2.cvtColor(red_frame, cv2.COLOR_BGR2GRAY)

        _, red_frame = cv2.threshold(red_frame, 127, 255, cv2.THRESH_BINARY)

        # find the contours in the frame
        contours, _ = cv2.findContours(red_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            # find the contour with the largest area
            largest_contour = max(contours, key=cv2.contourArea)

            # approximate the contour with a polygon
            epsilon = self.constants.TOLERANCE_DEFECT_POLYGON_CONTOUR * cv2.arcLength(largest_contour, True)
            approx = cv2.approxPolyDP(largest_contour, epsilon, True)

            # check if the polygon is a rectangle
            if len(approx) == 4:
                # check if the aspect ratio of the rectangle is outside of the interval [0.95, 1.05]
                x, y, w, h = cv2.boundingRect(approx)

                            
                aspect_ratio = max(float(w) / h, float(h) / w)
                if aspect_ratio < (1-self.constants.SQUARE_ASPECT_RATIO_TOLERANCE) or aspect_ratio > (1+self.constants.SQUARE_ASPECT_RATIO_TOLERANCE):
                    print("Red rectangle detected! Press SPACE to continue")
                    return True
        return False

    def process_blue_video(self) -> bool:
        """
        Function that tries to detect a blue square in the video feed. We define a
        square as a cuadrilateral with an aspect ratio between 1-SQUARE_ASPECT_RATIO_TOLERANCE
        and 1+SQUARE_ASPECT_RATIO_TOLERANCE.

        Returns:
            bool: True if a blue square is detected, False otherwise.
        """
        frame = self.get_frame()
        cv2.imshow("frame", frame)

        frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(frame_HSV, self.constants.BLUE_LOWER_BOUND, self.constants.BLUE_UPPER_BOUND)
        blue_frame = cv2.bitwise_and(frame, frame, mask=mask)
        blue_frame = cv2.cvtColor(blue_frame, cv2.COLOR_BGR2GRAY)
        _, blue_binary = cv2.threshold(blue_frame, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(blue_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            epsilon = self.constants.TOLERANCE_DEFECT_POLYGON_CONTOUR * cv2.arcLength(largest_contour, True)
            approx = cv2.approxPolyDP(largest_contour, epsilon, True)
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = max(float(w) / h, float(h) / w)
                if aspect_ratio > (1-self.constants.SQUARE_ASPECT_RATIO_TOLERANCE) and aspect_ratio < (1+self.constants.SQUARE_ASPECT_RATIO_TOLERANCE):
                    print("Blue square detected! Press SPACE to continue")
                    return True

        return False

    def process_green_video(self) -> bool:
        """
        Function that tries to detect a green triangle in the video feed. We define a
        triangle as a polygon with 3 sides.
        
        Returns:
            bool: True if a green triangle is detected, False otherwise.
        """

        frame = self.get_frame()
        cv2.imshow("frame", frame)

        frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(frame_HSV, self.constants.GREEN_LOWER_BOUND, self.constants.GREEN_UPPER_BOUND)
        green_frame = cv2.bitwise_and(frame, frame, mask=green_mask)
        green_frame = cv2.cvtColor(green_frame, cv2.COLOR_BGR2GRAY)
        _, green_binary = cv2.threshold(green_frame, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(green_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            epsilon = self.constants.TOLERANCE_DEFECT_POLYGON_CONTOUR * cv2.arcLength(largest_contour, True)
            approx = cv2.approxPolyDP(largest_contour, epsilon, True)
            if len(approx) == 3:
                print("Green triangle detected! Press SPACE to continue")
                return True

        return False

    def process_yellow_video(self) -> bool:
        """
        Function that tries to detect a yellow hexagon in the video feed. We define a
        hexagon as a polygon with 6 sides.

        Returns:
            bool: True if a yellow hexagon is detected, False otherwise.
        """
        frame = self.get_frame()
        cv2.imshow("frame", frame)
        
        frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(frame_HSV, self.constants.YELLOW_LOWER_BOUND, self.constants.YELLOW_UPPER_BOUND)
        yellow_frame = cv2.bitwise_and(frame, frame, mask=yellow_mask)
        yellow_frame = cv2.cvtColor(yellow_frame, cv2.COLOR_BGR2GRAY)
        _, yellow_binary = cv2.threshold(yellow_frame, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(yellow_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            epsilon = self.constants.TOLERANCE_DEFECT_POLYGON_CONTOUR * cv2.arcLength(largest_contour, True)
            approx = cv2.approxPolyDP(largest_contour, epsilon, True)

            if len(approx) == 6:
                print("Yellow hexagon detected! Press SPACE to end the program")
                return True

        return False

    def process_video(self) -> bool:
        """
        Function in charge of handling the processing of the video image. This function will
        call the appropriate function depending on the color sequence index (ie, the color of the shape
        we are looking for).

        Returns:
            bool: False if the sequence is complete (the last shape has been detected), True otherwise.
        """
        detected_shape = False
        if self.color_sequence_index == 0:
            print("Looking for red rectangle")
            detected_shape = self.process_red_video()
        
        if self.color_sequence_index == 1:
            print("Looking for blue square")
            detected_shape = self.process_blue_video()
        
        if self.color_sequence_index == 2:
            print("Looking for green triangle")
            detected_shape = self.process_green_video()

        if self.color_sequence_index == 3:
            print("Looking for yellow hexagon")
            detected_shape = self.process_yellow_video()
        
        if detected_shape:
            if self.color_sequence_index != 3:
                print("Press SPACE to continue")
                cv2.waitKey(0)
                self.next_color()
                self.last_object_detected_time = time()
            else:
                print("Sequence complete!")
                return False
            
        
        # reset the sequence if no shape is detected in TIME_TO_DETECT seconds
        if self.color_sequence_index > 0 : #and self.color_sequence_index < 4
            if time() - self.last_object_detected_time > self.constants.TIME_TO_DETECT:
                print("Timed out! Resetting sequence")
                self.reset_color_sequence()

        return True
     


if __name__ == "__main__":
    cam = Camera(type="webcam") # type can be either "pi" or "webcam"
    frame = cam.get_frame()
    cv2.imshow("frame", frame)
    
    main_loop = True
    while main_loop:
        cv2.imshow("frame", frame)
        main_loop = cam.process_video() # this function will return False if the sequence is complete

        # if the user presses the 'q' key, end the program
        if cv2.waitKey(1) & 0xFF == ord('q'):
            main_loop = False
            

    cam.end_program()