#!/usr/bin/env python

# Python libs
import sys
import time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from assignment2.msg import Ball_state
from assignment2.msg import HeadState

VERBOSE = False



class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        rospy.init_node('ballDetection', anonymous=True)
     # topic where we publish
        ## Publisher that send the processed and compressed images 
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        ## Publisher for send to the command manager information  reguarding the ball and the corraction to apply to the robot 
        self.ball_state_pub = rospy.Publisher("ball_state",Ball_state,queue_size=1)


        ## Publisher to move the robot 
        self.vel_pub = rospy.Publisher("cmd_vel",Twist, queue_size=1)

        # subscribed Topic
	## To get the compressed images from the camera  
        self.camera_sub = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, self.callback, queue_size=1)

	### Subscriber to the head_state topic to know when the command manager has finished moving his head
	self.head_state_sub = rospy.Subscriber("head_state",HeadState, self.HeadStateCallback, queue_size = 1)

	## Variable to check if the head is in the upright position
	self.headState = True

	## Variable that says whether the robot has reached the ball
	self.ballReached = False 


    def HeadStateCallback(self, data): 
        '''Callback function of subscribed "head_state" topic.
	 This function updates the "headState" value'''
	self.headState = data.HeadMotionStop

    def callback(self, ros_data):
        '''Callback function of subscribed "camera_sub" topic which implements 
        the whole Computer Vision algorithm  '''
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)

        #### direct conversion to CV2 ####
        ## @param image_np is the image decompressed and converted in OpendCv
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

	## set colours bound
        greenLower = (50, 50, 20)
        greenUpper = (70, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        #cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        # only proceed if at least one contour was found

        # fare l else che metta il messaggio ObjDet = false
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                
                msg = Ball_state()
		## Field of the ROS msg Ball_state 
                msg.ballDetected = True
		msg.ballReached = self.ballReached 
	        
                self.ball_state_pub.publish(msg)
		# check if the robot is reached the object 

        	 
		if self.headState == True: # if the head is not moving 	
			rospy.loginfo("BallDtection: ball detected !!! start moving ") 
		        vel = Twist()
		        # 400 is the center of the image 
		        vel.angular.z = -0.002*(center[0]-400)
			# 150 is the radius that we want see in the image, which represent the desired disatance from the object 
		        vel.linear.x = -0.01*(radius-150)
		        self.vel_pub.publish(vel)
			self.ballReached = False 
			
			if (radius>=148) and abs(center[0]-400)<5: #Condition for considering the ball as reached
				rospy.loginfo("BallDetection : ball reached!!")
				self.headState = False
				self.ballReached = True 
				msg.ballReached = self.ballReached		
				self.ball_state_pub.publish(msg) 

			

        else:
	     msg = Ball_state()
             msg.ballDetected = False 
             self.ball_state_pub.publish(msg)
            

        # update the points queue
        # pts.appendleft(center)
        cv2.imshow('window', image_np)
        cv2.waitKey(2)

        # self.subscriber.unregister()


def main(args):
    '''Initializes and cleanup ros node'''
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
