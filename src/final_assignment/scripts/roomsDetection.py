#!/usr/bin/env python
import sys
import time 

import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

from sensor_msgs.msg import CompressedImage 

ROOMS = [ 
    {'name':"LeavingRoom",'colour': "blue", "x":"", "y":""},
    {'name':"BedRoom",'colour':"red","x":"","y":""}
]



class roomDetector():
    def __init__(self):
	'''Initialize ros publisher, ros subscriber'''
        rospy.init_node('roomsDetection', anonymous=True)

        self.COLORS_VISITED = []
        # Subsriber to get the compressed images from the camera
        self.camera_sub = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, self.find_ball, queue_size=1)


    def ball_detect(self, hsv_min, hsv_max, image_np):
        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, hsv_min, hsv_max)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        if len(cnts) > 0:
            return True


    def find_ball(self, ros_image):
        #### direct conversion to CV2 ####
        ## @param image_np is the image decompressed and converted in OpendCv
        np_arr = np.fromstring(ros_image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  

        blackLower = (0, 0, 0)
        blackUpper = (5,50,50)
        redLower = (0, 50, 50)
        redUpper = (5, 255, 255)
        yellowLower = (25, 50, 50)
        yellowUpper = (35, 255, 255)
        greenLower = (50, 50, 50)
        greenUpper = (70, 255, 255)
        blueLower = (100, 50, 50)
        blueUpper = (130, 255, 255)
        magentaLower = (125, 50, 50)
        magentaUpper = (150, 255, 255)

        # black test
	x = self.ball_detect(blackLower, blackUpper, image_np)
        if x == True:
            rospy.loginfo("black ball detected ")
            if "black" not in self.COLORS_VISITED:
                print("New room detected!!!")
                self.COLORS_VISITED.append("black")
        # red test 
	x = self.ball_detect(redLower, redUpper, image_np)
        if x == True:
            rospy.loginfo("red ball detected")
            if "red" not in self.COLORS_VISITED:
                print("New room detected!!!")
                self.COLORS_VISITED.append("red")
        # yellow test
	x = self.ball_detect(yellowLower, yellowUpper, image_np)
        if x == True:
            rospy.loginfo("yellow ball detected")
            if "yellow" not in self.COLORS_VISITED:
                print("New room detected!!!")
                self.COLORS_VISITED.append("yellow")
        # green test
	x = self.ball_detect(greenLower, greenUpper, image_np)
        if x == True:
            rospy.loginfo("green ball detected")
            if "green" not in self.COLORS_VISITED:
                print("New room detected!!!")
                self.COLORS_VISITED.append("green")
        # blue test
	x = self.ball_detect(blueLower, blueUpper, image_np)
        if x == True:
            rospy.loginfo("blue ball detected")
            if "blue" not in self.COLORS_VISITED:
                print("New room detected!!!")
                self.COLORS_VISITED.append("blue")
        # magenta test
	x = self.ball_detect(magentaLower, magentaUpper, image_np)
        if x == True:
            rospy.loginfo("magenta ball detected")
            if "magenta" not in self.COLORS_VISITED:
                print("New room detected!!!")
                self.COLORS_VISITED.append("magenta")

def main(args):
    rd = roomDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

