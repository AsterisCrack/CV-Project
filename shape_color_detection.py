import cv2
import numpy as np
from time import sleep
from picamera2 import Picamera2

# define the lower and upper bounds of the colors in the HSV color space
# TO DO: give specific (hue, saturation, value) values to the lower and upper bounds
RED_LOWER_BOUND_1 = np.ndarray((0, 0, 0))
RED_UPPER_BOUND_1 = np.ndarray((0, 0, 0)) 
RED_LOWER_BOUND_2 = np.ndarray((0, 0, 0)) 
RED_UPPER_BOUND_2 = np.ndarray((0, 0, 0)) 
"""
Because red is the color at the beginning/end of the color circle in the HSV
color space, we may want to define two ranges for red, one for the beginning
and one for the end of the color circle, so that we can detect colors at either end
of the color spectrum.
"""

BLUE_LOWER_BOUND = np.ndarray((0, 0, 0)) 
BLUE_UPPER_BOUND = np.ndarray((0, 0, 0)) 

GREEN_LOWER_BOUND = np.ndarray((0, 0, 0))
GREEN_UPPER_BOUND = np.ndarray((0, 0, 0))

YELLOW_LOWER_BOUND = np.ndarray((0, 0, 0))
YELLOW_UPPER_BOUND = np.ndarray((0, 0, 0))

SQUARE_ASPECT_RATION_TOLERANCE = 0.05 # how far off the aspect ratio of a cuadrilateral
# can be from 1.0 (a square) and still be considered a square. 

TIME_BETWEEN_DETECTIONS = 2 # the time in seconds between detections of the pattern
# of colored shapes. If the program detects the pattern, it will wait this amount of time
# before checking again if the pattern is still there. If the pattern is not there anymore,
# the program will continue running. If the pattern is still there, the program will stop
# and print a message saying that the pattern has been detected.


def red_object_detection(frame: np.ndarray) -> bool:
    """
    This function detects red objects in the video stream.
    The red object it must detect is a red rectangle (we define a rectangle as
    a figure with 4 sides, with an aspect ratio outside of the interval [0.95, 1.05]).

    Args:
        frame: the frame of the video stream to analyze

    Returns:
        bool: True if the red rectangle has been detected, False otherwise

    """

    HSV_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # convert the frame to the HSV color space

    # create a mask for the red color
    red_mask_1 = cv2.inRange(HSV_frame, RED_LOWER_BOUND_1, RED_UPPER_BOUND_1)
    red_mask_2 = cv2.inRange(HSV_frame, RED_LOWER_BOUND_2, RED_UPPER_BOUND_2)
    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

    # apply the mask to the frame
    red_frame = cv2.bitwise_and(frame, frame, mask=red_mask)

    # convert the frame to grayscale
    red_frame = cv2.cvtColor(red_frame, cv2.COLOR_BGR2GRAY)

    # apply a threshold to the frame
    _, red_frame = cv2.threshold(red_frame, 127, 255, cv2.THRESH_BINARY)

    # find the contours of the red objects
    contours, _ = cv2.findContours(red_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # find the contour with the largest area
    largest_contour = max(contours, key=cv2.contourArea)

    # approximate the contour with a polygon
    epsilon = 0.1 * cv2.arcLength(largest_contour, True)
    approx = cv2.approxPolyDP(largest_contour, epsilon, True)

    # check if the polygon is a rectangle
    if len(approx) == 4:
        # check if the aspect ratio of the rectangle is outside of the interval [0.95, 1.05]
        x, y, w, h = cv2.boundingRect(approx)
        aspect_ratio = max(float(w) / h, float(h) / w)
        if aspect_ratio < 0.95 or aspect_ratio > 1.05:

            return True
    

    return False


def blue_object_detection(frame: np.ndarray) -> bool:
    """
    This function detects blue objects in the video stream.
    The blue object it must detect is a blue square (we define a square as
    a figure with 4 sides, with an aspect ratio inside of the interval [0.95, 1.05]).

    Args:
        frame (np.ndarray): the frame of the video stream to analyze

    Returns:
        bool: True if the blue square has been detected, False otherwise
    """

    HSV_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # convert the frame to the HSV color space

    # create a mask for the blue color
    blue_mask = cv2.inRange(HSV_frame, BLUE_LOWER_BOUND, BLUE_UPPER_BOUND)

    # apply the mask to the frame
    blue_frame = cv2.bitwise_and(frame, frame, mask=blue_mask)

    # convert the frame to grayscale
    blue_frame = cv2.cvtColor(blue_frame, cv2.COLOR_BGR2GRAY)

    # apply a threshold to the frame
    _, blue_frame = cv2.threshold(blue_frame, 127, 255, cv2.THRESH_BINARY)

    # find the contours of the blue objects
    contours, _ = cv2.findContours(blue_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # find the contour with the largest area
    largest_contour = max(contours, key=cv2.contourArea)

    # approximate the contour with a polygon
    epsilon = 0.1 * cv2.arcLength(largest_contour, True)
    approx = cv2.approxPolyDP(largest_contour, epsilon, True)

    # check if the polygon is a square
    if len(approx) == 4:
        # check if the aspect ratio of the square is inside of the interval [0.95, 1.05]
        x, y, w, h = cv2.boundingRect(approx)
        aspect_ratio = max(float(w) / h, float(h) / w)
        if aspect_ratio > 0.95 and aspect_ratio < 1.05:

            return True

    return False


def green_object_detection(frame: np.ndarray) -> bool:
    """
    This function detects green objects in the video stream.
    The green object it must detect is a green triangle (we define a triangle as
    a figure with 3 sides).

    Args:
        frame (np.ndarray): the frame of the video stream to analyze

    Returns:
        bool: True if the green triangle has been detected, False otherwise
    """

    HSV_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # convert the frame to the HSV color space

    # create a mask for the green color
    green_mask = cv2.inRange(HSV_frame, GREEN_LOWER_BOUND, GREEN_UPPER_BOUND)

    # apply the mask to the frame
    green_frame = cv2.bitwise_and(frame, frame, mask=green_mask)

    # convert the frame to grayscale
    green_frame = cv2.cvtColor(green_frame, cv2.COLOR_BGR2GRAY)

    # apply a threshold to the frame
    _, green_frame = cv2.threshold(green_frame, 127, 255, cv2.THRESH_BINARY)

    # find the contours of the green objects
    contours, _ = cv2.findContours(green_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # find the contour with the largest area
    largest_contour = max(contours, key=cv2.contourArea)

    # approximate the contour with a polygon
    epsilon = 0.1 * cv2.arcLength(largest_contour, True)
    approx = cv2.approxPolyDP(largest_contour, epsilon, True)

    # check if the polygon is a triangle
    if len(approx) == 3:

        return True

    return False


def yellow_object_detection(frame: np.ndarray) -> bool:
    """
    This function detects yellow objects in the video stream.
    The yellow object it must detect is a yellow hexagon (we define a hexagon as
    a figure with 6 sides).

    Args:
        frame (np.ndarray): the frame of the video stream to analyze

    Returns:
        bool: True if the yellow hexagon has been detected, False otherwise
    """

    HSV_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # convert the frame to the HSV color space

    # create a mask for the yellow color
    yellow_mask = cv2.inRange(HSV_frame, YELLOW_LOWER_BOUND, YELLOW_UPPER_BOUND)

    # apply the mask to the frame
    yellow_frame = cv2.bitwise_and(frame, frame, mask=yellow_mask)

    # convert the frame to grayscale
    yellow_frame = cv2.cvtColor(yellow_frame, cv2.COLOR_BGR2GRAY)

    # apply a threshold to the frame
    _, yellow_frame = cv2.threshold(yellow_frame, 127, 255, cv2.THRESH_BINARY)

    # find the contours of the yellow objects
    contours, _ = cv2.findContours(yellow_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # find the contour with the largest area
    largest_contour = max(contours, key=cv2.contourArea)

    # approximate the contour with a polygon
    epsilon = 0.1 * cv2.arcLength(largest_contour, True)
    approx = cv2.approxPolyDP(largest_contour, epsilon, True)

    # check if the polygon is a hexagon
    if len(approx) == 6:

        return True

    return False





def stream_video():
    picam = Picamera2()
    picam.preview_configuration.main.size=(1280, 720)
    picam.preview_configuration.main.format="RGB888"
    picam.preview_configuration.align()
    picam.configure("preview")
    picam.start()

    while True:
        frame = picam.capture_array()

        cv2.imshow("picam", frame)

        """
        We want to make so that the program stops when it detects a certain pattern of colored shapes
        in the video stream. We will use the following pattern:
        - A red rectangle
        - A blue square
        - A green triangle
        - A yellow hexagon
        
        If at any point the program detects this pattern, it will stop.
        and print a message saying that the pattern has been detected.

        If the program does not detect the pattern, it will continue running until it does.
        If it starts detecting the pattern, but then it stops detecting it, it will continue running

        The program will give a 2 second clearance time to the user to move the shapes around
        and make the program stop detecting the pattern. If the program does not detect the pattern

        """
        # check if the red rectangle has been detected
        red_rectangle_detected = red_object_detection(frame)
        
        if red_rectangle_detected:
            print("Red rectangle detected")
            # wait 2 seconds
            sleep(TIME_BETWEEN_DETECTIONS)
            blue_object_detection(frame)
            if blue_object_detection(frame):
                print("Blue square detected")
                # wait 2 seconds
                sleep(TIME_BETWEEN_DETECTIONS)
                green_object_detection(frame)
                if green_object_detection(frame):
                    print("Green triangle detected")
                    # wait 2 seconds
                    sleep(TIME_BETWEEN_DETECTIONS)
                    yellow_object_detection(frame)
                    if yellow_object_detection(frame):
                        print("Yellow hexagon detected")

                        print("\n\n\tFULL PATTERN DETECTED.")
                        break
                    else:
                        print("Wrong shape detected. Restarting detection process.")
                else:
                    print("Wrong shape detected. Restarting detection process.")
            else:
                print("Wrong shape detected. Restarting detection process.")



        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    # release the camera
    picam.stop()







if __name__ == "__main__":
    stream_video()
    