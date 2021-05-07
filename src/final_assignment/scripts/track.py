#!/usr/bin/env python

## @file track.py
#  This script implements an action server which is able to track a ball of a given color.  
 

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
from sensor_msgs.msg import LaserScan

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




## Class that implements the action server for tracking the ball. 
# It takes a color of a ball in proximity of the robot and start to track it.
# Periodically the robot store its own position (through a subscriber to the /odom topic)in a variable in order to return its location when a ball will be reached.
# It may happen that the robot is no longer able to detect the ball, in this case the robot will rotate around itself (right and left) in order to find the ball again. 
# If not the mission will be aborted.
# Finally it's also able to avoid simple obstacle in front, front-right and front-left directions with respect to the robot
class TrackAction(object): 
             
    def __init__(self, name):
	## Name of the action server
        self.actionName = name
        self.act_s = actionlib.SimpleActionServer('trackAction', final_assignment.msg.trackBallAction, self.track, auto_start=False)
        self.act_s.start()
	self.feedback = final_assignment.msg.trackBallFeedback()
    	self.result = final_assignment.msg.trackBallResult()
        ## Dictionary which defines the laser data perceived in the following 5 directions 
        self.regions = {
                'right': 0,
                'fright': 0,
                'front': 0,
                'fleft': 0,
                'left': 0,
            }

        ## Flag that notifies that the robot has reached the ball correctly
        self.success = False
        ## Variables that counts how many times the robot was not able to detect the ball again.
        # After which it abort the mission and close the action server. 
        self.unfound_ball_counter = 0
        ## Flag that notifies aborting mission, since the robot is not able to detect the ball or if there is no way to avoid an obstacle
        self.abort = False
        ## Radius of the detected ball designed by the openCV algorithm in the reach_ball function
        self.radius = 0
        ## Publisher to the /cmd_vel topic in order to move the robot towards tha balls
        self.vel_pub = 0
        ## Variable that store the current position of the robot 
        self.position = Point()
        ## Current pose of the robot
        self.pose = Pose()
        
    ## Callback to the /odom topic which updates the global variables: position and pose of the robot
    def clbk_odom(self, msg):
        self.position = msg.pose.pose.position
        self.pose = msg.pose.pose
        


    ## Callback function of the camera subscriber which computes the velocities to apply to the robot until it reaches the target ball.
    ## @param ros_image is the image decompressed and converted in OpendCv
    def reach_ball(self, ros_image):
        #### direct conversion to CV2 ####        
        np_arr = np.fromstring(ros_image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  

        # Bluring the image to reduce noise
        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        ## Conversion into HSV format to easily finding contours
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

        # find the countours
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        center = None
        # only proceed if at least one contour was found

        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), self.radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if self.radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(self.radius), (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)

                # Setting the velocities to be applied to the robot
                vel = Twist()
		# 400 is the center of the camera image, therefore it's used to compute the correction along z
		vel.angular.z = -0.002*(center[0]-400)
		 # 150 is the radius that we want see in the image, which represent the desired distance from the object 
		vel.linear.x = -0.007*(self.radius-150)
        # apply the velocities
		self.vel_pub.publish(vel)
        #Condition for considering the ball as reached
                if (self.radius>=143) and abs(center[0]-400)<5: 
				rospy.loginfo("[Track] ball reached!!")
                # set the returning values as the last robot location perceived
                		self.result.x = self.position.x
                		self.result.y = self.position.y
				self.success = True

        else:
            # Routine to find tha ball again
            rospy.loginfo("[Track] Ball not found")
	    vel = Twist()
	    if self.unfound_ball_counter <= 8:
		 rospy.loginfo("[Track] Turn Right to find the ball")
                 vel.angular.z = 0.5
		 self.vel_pub.publish(vel) 
	    elif self.unfound_ball_counter < 17:
		 rospy.loginfo("[Track] Turn Left to find the ball")
                 vel.angular.z = -0.5
		 self.vel_pub.publish(vel)
	    elif self.unfound_ball_counter == 17:
		 rospy.loginfo("[Track] Robot is unable to find the ball ")
		 self.unfound_ball_counter = 0
		 self.abort = True   
	    self.unfound_ball_counter += 1

    ## Callback function of the LaserScan topic which implements a basic obstacle avoidance algorithm.
    # Basically it just evaluate the distance to an obstacle considering only 3 direction (front, front-left, front-right), then chooses a proper correation to applay.
    # @param msg LaserScan message to get the distances from the obstacles
    def obstacle_avoidance(self, msg):
        vel = Twist()
        self.regions = {
                'right':  min(min(msg.ranges[0:143]), 10),
                'fright': min(min(msg.ranges[144:287]), 10),
                'front':  min(min(msg.ranges[288:431]), 10),
                'fleft':  min(min(msg.ranges[432:575]), 10),
                'left':   min(min(msg.ranges[576:713]), 10),
            }
        threshold = 0.6
        threshold2 = 0.85
        if (self.regions['front'] > 0) and (self.regions['front'] <= threshold) and (self.radius < 110):
            rospy.loginfo("[Track] obstacle detected ")
            if self.regions['fright'] > threshold2:
                rospy.loginfo("[Track] turn right")
                vel.angular.z = -0.5
                self.vel_pub.publish(vel)
            elif self.regions['fleft'] > threshold2:
                rospy.loginfo("[Track] turn left")
                vel.angular.z = 0.5
                self.vel_pub.publish(vel)
            else:
                rospy.loginfo("[Track] aborted")
                self.abort = True 
                
        
	    
    ## Action server routine function 
    # @param goal Contains the color of the ball that has been detected     
    def track(self, goal):
        self.color = goal.color
	## Odom subcriber to get and save the robot position and send them back to the commandManager 
        sub_odom = rospy.Subscriber('odom', Odometry, self.clbk_odom)
	## Publisher to the cmd_vel topic in order to move the robot
        self.vel_pub = rospy.Publisher("cmd_vel",Twist, queue_size=1)
        ## Subscriber to camera1 in order to receive and handle the images
        camera_sub = rospy.Subscriber("camera1/image_raw/compressed", CompressedImage, self.reach_ball, queue_size=1)
        ## Subscriber to the laser scan to perceive obstacle distances
        sub_scan = rospy.Subscriber('/scan', LaserScan, self.obstacle_avoidance)	 
        
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
	# Unregister from any topic since the action server is going to be closed
	camera_sub.unregister()
	sub_scan.unregister()
	sub_odom.unregister()
	self.vel_pub.unregister()
    	
	if not self.abort == True:
        # If the mission was not aborted returns the result
	     self.act_s.set_succeeded(self.result)	     
	else:
        # Otherwise preampt the action server 
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
