#!/usr/bin/env python

## @file track.py
#  This script is an acttion server which takes a color of a ball in proximity of the robot and start to track it. It may happen that the robot is no longer able to detect the ball, in this case the robot will rotate around itself (right and left) in order to find the ball again. If not the mission will be aborted
 

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
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry

# Action Server 
import actionlib
import actionlib.msg
import final_assignment.msg

# Colors Thresholds 
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

## Variable of type Point which stores the current odometry position of the robot
position_ = Point()
## Variable of type Pose which stores the current odometry position of the robot 
pose_ = Pose()


## Callback to the Odom topic which updates the global variables: position_ and pose_
def clbk_odom(msg):
    global position_
    global pose_

    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose




## Class that implements the action server for tracking the ball
class TrackAction(object): 
             
    def __init__(self, name):
	## Name of the action server
        self.actionName = name
        self.act_s = actionlib.SimpleActionServer('trackAction', final_assignment.msg.trackBallAction, self.track, auto_start=False)
        self.act_s.start()
	self.feedback = final_assignment.msg.trackBallFeedback()
    	self.result = final_assignment.msg.trackBallResult()

        ## Flag that notifies that the robot has reached the ball correctly
	self.success = False
	## Variables that counts how many times the robot was not able to detect the ball again. 
	self.unfound_ball_counter = 0
	## Flag that notifies aborting mission since the robot was not able to detect the ball 
	self.abort = False


    ## Callback function of the camera subscriber which computes the velocities to apply to the robot untill it reaches the ball.
    ## @param ros_image is the image decompressed and converted in OpendCv
    def reach_ball(self, ros_image):
        global position_, pose_
        #### direct conversion to CV2 ####        
        np_arr = np.fromstring(ros_image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Apply the proper color mask
        if self.color == "black":
            mask = cv2.inRange(hsv, blackLower, blackUpper)
        elif self.color == "red":
            mask = cv2.inRange(hsv, redLower, redUpper)
        elif self.color == "yellow":
            mask = cv2.inRange(hsv, yellowLower, yellowUpper)
        elif self.color == "green":
            mask = cv2.inRange(hsv, greenLower, greenUpper)
        elif self.color == "blue":
            mask = cv2.inRange(hsv, blueLower, blueUpper)
        elif self.color == "magenta":
            mask = cv2.inRange(hsv, magentaLower, magentaUpper)
        
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        center = None
        # only proceed if at least one contour was found

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
                cv2.circle(image_np, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)

                # Setting the velocities to be applied to the robot
                vel = Twist()
		# 400 is the center of the image 
		vel.angular.z = -0.002*(center[0]-400)
		 # 150 is the radius that we want see in the image, which represent the desired disatance from the object 
		vel.linear.x = -0.007*(radius-150)
		self.vel_pub.publish(vel)

                if (radius>=148) and abs(center[0]-400)<5: #Condition for considering the ball as reached
				rospy.loginfo("[Track] ball reached!!")
                		self.result.x = position_.x
                		self.result.y = position_.y
				self.success = True

        else:
            rospy.loginfo("[Track] Ball not found")
	    vel = Twist()
	    if self.unfound_ball_counter <= 10:
		 rospy.loginfo("[Track] Turn Right to find the ball")
                 vel.angular.z = 1.5
		 self.vel_pub.publish(vel) 
	    elif self.unfound_ball_counter < 20:
		 rospy.loginfo("[Track] Turn Left to find the ball")
                 vel.angular.z = -1.5
		 self.vel_pub.publish(vel)
	    elif self.unfound_ball_counter == 20:
		 rospy.loginfo("[Track] Robot is unable to find the ball ")
		 self.unfound_ball_counter = 0
		 self.abort = True   
	    self.unfound_ball_counter += 1
	    
    ## Action server routine function 
    # @param goal Contains the color of the ball that has been detected     
    def track(self, goal):
        self.color = goal.color
	## Odom subcriber to get and save the robot position and send them back to the commandManager 
        sub_odom = rospy.Subscriber('odom', Odometry, clbk_odom)
	## Publisher to the cmd_vel topic in order to move the robot
        self.vel_pub = rospy.Publisher("cmd_vel",Twist, queue_size=1)
        ## Subscriber to camera1 in order to recive and handle the images
        camera_sub = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, self.reach_ball, queue_size=1)	 
        
        while not self.success:
            if self.act_s.is_preempt_requested():
                rospy.loginfo('[Track] Goal was preempted')
                self.act_s.set_preempted()
                break
            elif self.abort == True:
		rospy.loginfo('[Track] Mission aborted')
		break
	    else:
                self.feedback.state = "Reaching the ball..."
		self.act_s.publish_feedback(self.feedback)
	# Unregister from the odom and camera topic
	camera_sub.unregister()
	sub_odom.unregister()
	self.vel_pub.unregister()
	if not self.abort == True:
	     self.act_s.set_succeeded(self.result)
	     
	else:

	     self.act_s.set_preempted()
	self.abort = False
	self.success = False
	rospy.loginfo("[Track] action server closed")
	
	    
		

def main():
    try:
        rospy.init_node('Track')

        track = TrackAction('TrackAction')

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        print ("Shutting down Track action server module")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
