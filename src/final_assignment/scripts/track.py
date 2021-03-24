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
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry

# Action Server 
import actionlib
import actionlib.msg
import final_assignment.msg


# Colors
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


position_ = Point()
pose_ = Pose()

## Odom sub callback
def clbk_odom(msg):
    global position_
    global pose_
    global yaw_

    # position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

    '''
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]'''




class TrackAction(object): # forse ci va il goal
    feedback = final_assignment.msg.trackBallFeedback()
    result = final_assignment.msg.trackBallResult()
    
    ## Publisher to move the robot 
    
    def __init__(self, name):
        self.actionName = name
        self.act_s = actionlib.SimpleActionServer('robot_reach_room', final_assignment.msg.trackBallAction, self.track, auto_start=False)
        self.act_s.start()
	self.vel_pub = rospy.Publisher("cmd_vel",Twist, queue_size=1)
	rate = rospy.Rate(20)

    	while not rospy.is_shutdown():
        	rate.sleep()
        # posso mettere qui volendo il sub scriber ad odom 


    def reach_ball(self, ros_image):
        global position_, pose_
	
        #### direct conversion to CV2 ####
        ## @param image_np is the image decompressed and converted in OpendCv
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
		vel.linear.x = -0.005*(radius-150)
		self.vel_pub.publish(vel)

                if (radius>=148) and abs(center[0]-400)<5: #Condition for considering the ball as reached
			rospy.loginfo("BallDetection : ball reached!!")
                	self.result.x = position_.x
                	self.result.y = position_.y
			rospy.loginfo("the current robot position is:")
			rospy.loginfo(self.result.x)
			rospy.loginfo(self.result.y)
			self.act_s.set_succeeded(self.result)
			# unregister from the camera topic in order to stop to track the ball
			self.camera_sub.unregister()
			self.state = 1
			#break

        else:
            rospy.loginfo("Ball not found")

    def done(self):
	twist_msg = Twist()
    	twist_msg.linear.x = 0
    	twist_msg.angular.z = 0
	self.vel_pub.publish(twist_msg)
        
    def track(self, goal):
        self.color = goal.color
	rospy.loginfo("Action server created ")
        # crate the subscriber to camera1 in order to recive and handle the images
        self.camera_sub = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, self.reach_ball, queue_size=1)
        
	self.state = 0
        while not rospy.is_shutdown():
            if self.act_s.is_preempt_requested():
                rospy.loginfo('Goal was preempted')
                self.act_s.set_preempted()
                success = False
                break
            elif self.state == 0:
                self.feedback = "Reaching the ball"
	    elif self.state == 1:
		rospy.loginfo("goal reached!")
		self.done()
		break




def main():
    rospy.init_node('Track')

    # Odom sub to get and save the robot position when it reachs the ball 
    sub_odom = rospy.Subscriber('odom', Odometry, clbk_odom)

    track = TrackAction('TrackAction')



if __name__ == '__main__':
    main()