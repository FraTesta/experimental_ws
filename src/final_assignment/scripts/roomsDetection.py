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
from std_msgs.msg import String



class roomDetector():
    def __init__(self):
	'''Initialize ros publisher, ros subscriber'''
        rospy.init_node('roomsDetection', anonymous=True)

        #self.COLORS_VISITED = []
	## Contains the previous color detected
	self.prevColor = 'None'
        # Subsriber to get the compressed images from the camera
        self.camera_sub = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, self.find_ball, queue_size=1)
        self.newRoomPub = rospy.Publisher('newRoom', String, queue_size=10)
	self.rate = rospy.Rate(1)
	self.ballDetected = False

    # returns True when a ball is detected
    def find_color(self, hsv_min, hsv_max, image_np):
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
	
	self.ballDetected = False 

        # black test
	self.ballDetected = self.find_color(blackLower, blackUpper, image_np)
        if self.ballDetected == True:
            rospy.loginfo("[roomsDetection] black ball detected ")
            if "black" != self.prevColor:               
                self.prevColor = "black"
		rospy.loginfo("[roomsDetection] New room detected!!!")
                self.newRoomPub.publish("black")
        # red test 
	self.ballDetected = self.find_color(redLower, redUpper, image_np)
        if self.ballDetected == True:
            rospy.loginfo("[roomsDetection] red ball detected")
            if "red" != self.prevColor:
                rospy.loginfo("[roomsDetection] detected!!!")
                self.prevColor = "red"
                self.newRoomPub.publish("red")
        # yellow test
	self.ballDetected = self.find_color(yellowLower, yellowUpper, image_np)
        if self.ballDetected == True:
            rospy.loginfo("[roomsDetection] yellow ball detected")
            if "yellow" != self.prevColor:
                rospy.loginfo("New room detected!!!")
                self.prevColor = "yellow"
                self.newRoomPub.publish("yellow")
        # green test
	self.ballDetected = self.find_color(greenLower, greenUpper, image_np)
        if self.ballDetected == True:
            rospy.loginfo("[roomsDetection] green ball detected")
            if "green" != self.prevColor:
                rospy.loginfo("New room detected!!!")
                self.prevColor = "green"
                self.newRoomPub.publish("green")
        # blue test
	self.ballDetected = self.find_color(blueLower, blueUpper, image_np)
        if self.ballDetected == True:
            rospy.loginfo("[roomsDetection] blue ball detected")
            if "blue" != self.prevColor:
                
                self.prevColor = "blue"
		rospy.loginfo("New room detected!!!")
                self.newRoomPub.publish("blue")
        # magenta test
	self.ballDetected = self.find_color(magentaLower, magentaUpper, image_np)
        if self.ballDetected == True:
            rospy.loginfo("[roomsDetection] magenta ball detected")
            if "magenta" != self.prevColor:
                rospy.loginfo("New room detected!!!")
                self.prevColor = "magenta"
                self.newRoomPub.publish("magenta")
	cv2.imshow('window', image_np)
	#self.rate.sleep()
	cv2.waitKey(2)

def main(args):
    
    try:
	# delay in order to wait the commandManager launch
	time.sleep(12)
        rd = roomDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        print ("[roomsDetection] Shutting down roomDetection module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

