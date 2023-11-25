import numpy as np
import cv2
import sys
import time
import argparse
from collections import deque

#Construct the argument parser and parse the arguments
ap=argparse.ArgumentParser()
ap.add_argument("-v","--video",help="path to the (optional) video file")
ap.add_argument("-b","--buffer",type=int,default=64,help="max buffer size")
args=vars(ap.parse_args())

#We will use a red ball for tracking
#Lower and upper bounds for the red ball
lowerBound=np.array([0,100,100])
upperBound=np.array([10,255,255])
pts=deque(maxlen=args["buffer"])

if not args.get("video",False):
    camera=cv2.VideoCapture(0)
else:
    camera=cv2.VideoCapture(args["video"])

#Wait till the camera is ready
time.sleep(2.0)

#Ball tracking
while True:
    #Grab the current frame
    (grabbed,frame)=camera.read()

    #If we are viewing a video and we did not grab a frame,then we have reached the end of the video
    if args.get("video") and not grabbed:
        break

    #Resize the frame,blur it and convert it to the HSV color space
    frame=cv2.resize(frame,(600,400))
    blurred=cv2.GaussianBlur(frame,(11,11),0)
    hsv=cv2.cvtColor(blurred,cv2.COLOR_BGR2HSV)

    #Construct a mask for the color "red",then perform a series of dilations and erosions to remove any small blobs left in the mask
    mask=cv2.inRange(hsv,lowerBound,upperBound)
    mask=cv2.erode(mask,None,iterations=2)
    mask=cv2.dilate(mask,None,iterations=2)

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
        if radius>10:
            #Draw the circle and centroid on the frame,then update the list of tracked points
            cv2.circle(frame,(int(x),int(y)),int(radius),(0,255,255),2)
            cv2.circle(frame,center,5,(0,0,255),-1)

    #Update the points queue
    pts.appendleft(center)

    #Loop over the set of tracked points
    for i in range(1,len(pts)):
        #If either of the tracked points are None,ignore them
        if pts[i-1] is None or pts[i] is None:
            continue

        #Otherwise,compute the thickness of the line and draw
        thickness=int(np.sqrt(args["buffer"]/float(i+1))*2.5)
        cv2.line(frame,pts[i-1],pts[i],(0,0,255),thickness)

    #Show the frame to our screen
    #Flip it horizontally first for a mirror effect
    cv2.imshow("Frame",cv2.flip(frame,1))
    key=cv2.waitKey(1)&0xFF

    #If the 'q' key is pressed,stop the loop
    if key==ord("q"):
        break

#Cleanup the camera and close any open windows
camera.release()
cv2.destroyAllWindows()

