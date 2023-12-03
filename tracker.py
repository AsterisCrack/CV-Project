import numpy as np
import cv2
import sys
import time
import argparse
from collections import deque
from picamera2 import Picamera2

#Tracker class
class Tracker:
    def __init__(self, buffer=64, video=None, color='red', min_radius=10, using_raspberri=True, on_new_frame_function=None, camera=None):
        self.buffer = buffer
        self.video = video
        self.color = color
        self.min_radius = min_radius
        self.using_raspberri = using_raspberri
        self.on_new_frame_function = on_new_frame_function
        self.pts = deque(maxlen=self.buffer)
        if camera is not None:
            self.camera = camera
        else:
            self.init_camera()
    
    def init_camera(self):
        if not self.video:
            if self.using_raspberri:
                self.camera = Picamera2()
                self.camera.preview_configuration.main.size=(1280, 720)
                self.camera.preview_configuration.main.format="RGB888"
                self.camera.preview_configuration.align()
                self.camera.configure("preview")
                self.camera.start()
            else:
                self.camera = cv2.VideoCapture(0)
        else:
            self.camera = cv2.VideoCapture(self.video)
        time.sleep(2.0)

    def get_frame_normal(self):
        #Grab the current frame
        (grabbed,frame)=self.camera.read()

        #If we are viewing a video and we did not grab a frame,then we have reached the end of the video
        if self.video and not grabbed:
            return None

        #Resize the frame,blur it and convert it to the HSV color space
        frame=cv2.resize(frame,(600,400))
        blurred=cv2.GaussianBlur(frame,(11,11),0)
        hsv=cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)
        return hsv
    
    def get_frame_raspi(self):
        frame = self.camera.capture_array()
        frame=cv2.resize(frame,(600,400))
        blurred=cv2.GaussianBlur(frame,(11,11),0)
        hsv=cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)
        return hsv
    
    def get_mask(self, img):
        if self.color == 'red':
            #Red needs 2 masks because it gets cut off at 0
            lower1 = np.array([0, 100, 20])
            upper1 = np.array([10, 255, 255])

            lower2 = np.array([160,100,20])
            upper2 = np.array([179,255,255])

            lower_mask = cv2.inRange(img, lower1, upper1)
            upper_mask = cv2.inRange(img, lower2, upper2)
            mask = lower_mask + upper_mask

        elif self.color == 'blue':
            lower = np.array([100, 100, 20])
            upper = np.array([120, 255, 255])
            mask = cv2.inRange(img, lower, upper)

        elif self.color == 'green':
            lower = np.array([45, 100, 20])
            upper = np.array([76, 255, 255])
            mask = cv2.inRange(img, lower, upper)

        elif self.color == 'yellow':
            lower = np.array([20, 100, 20])
            upper = np.array([40, 255, 255])
            mask = cv2.inRange(img, lower, upper)

        else:
            raise ValueError('Invalid color. Color must be red, blue, green, or yellow.')
        
        mask = cv2.erode(mask,None,iterations=3)
        mask = cv2.dilate(mask,None,iterations=3)
        return mask
    
    def get_center(self, mask):
        #Find contours in the mask and initialize the current (x,y) center of the ball
        cnts=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center=None

        #Only proceed if at least one contour was found
        if len(cnts)>0:
            #Find the largest contour in the mask,then use it to compute the minimum enclosing circle and centroid
            c=max(cnts,key=cv2.contourArea)
            ((x,y),radius)=cv2.minEnclosingCircle(c)
            M=cv2.moments(c)
            center=(int(M["m10"]/M["m00"]),int(M["m01"]/M["m00"]))

            #Only proceed if the radius meets a minimum size
            if radius>self.min_radius:
                return center, radius, (x, y)
        return None, None, None
    
    def paint_frame(self, frame, center, radius, centroid):
        frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)
        #Draw the circle and centroid on the frame,then update the list of tracked points
        cv2.circle(frame,(int(centroid[0]),int(centroid[1])),int(radius),(0,255,255),2)
        cv2.circle(frame,center,5,(0,0,255),-1)

        #Loop over the set of tracked points
        for i in range(1,len(self.pts)):
            #If either of the tracked points are None,ignore them
            if self.pts[i-1] is None or self.pts[i] is None:
                continue

            #Otherwise,compute the thickness of the line and draw the connecting lines
            thickness=int(np.sqrt(self.buffer/float(i+1))*2.5)
            cv2.line(frame,self.pts[i-1],self.pts[i],(0,0,255),thickness)
        
        #Flip the frame horizontally
        frame = cv2.flip(frame, 1)
        return frame
    
    def track_single_frame(self):
        if self.using_raspberri:
            hsv = self.get_frame_raspi()
        else:
            hsv = self.get_frame_normal()
        mask = self.get_mask(hsv)
        center, radius, centroid = self.get_center(mask)
        if center is not None:
            self.pts.appendleft(center)
            if self.on_new_frame_function is not None:
                self.on_new_frame_function(centroid)
            return self.paint_frame(hsv, center, radius, centroid)
        return cv2.flip(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR), 1)
    
    def track(self):
        while True:
            frame = self.track_single_frame()
            if frame is None:
                break
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    def close(self):
        if self.using_raspberri:
            self.camera.stop()
        else:
            self.camera.release()
        cv2.destroyAllWindows()

    def __del__(self):
        self.close()

#Main
if __name__ == "__main__":
    #Construct the argument parse and parse the arguments
    ap=argparse.ArgumentParser()
    ap.add_argument("-rpi","--raspberri",type=bool,default=True,help="uses the raspberry pi camera")
    ap.add_argument("-v","--video",help="path to the (optional) video file")
    ap.add_argument("-b","--buffer",type=int,default=64,help="max buffer size")
    ap.add_argument("-c","--color",type=str,default='red',help="color to track")
    ap.add_argument("-r","--min-radius",type=int,default=10,help="minimum radius of the object to track")
    args=vars(ap.parse_args())
    
    tracker = Tracker(args['buffer'], args['video'], args['color'], args['min_radius'], args['raspberri'])
    tracker.track()
    tracker.close()


    



    
    



        