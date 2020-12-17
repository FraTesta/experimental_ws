#!/usr/bin/env python
from __future__ import print_function

import rospy 
import random
import time 
import actionlib
import actionlib_tutorials.msg
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkState
from assignment2.msg import Planning_ballAction, Planning_ballGoal, Planning_ballResult, Planning_ballFeedback





def randomTime():
         ''' function to let random time pass '''
	 rate = rospy.Rate(1)
	 randi = random.randint(3,8)
	 for i in range(randi):
		rate.sleep()


def main():
     rospy.init_node('random_ball_position ')
     ## Variable that caunts how many position has been reached up to now
     ballDessapear = 0
     client = actionlib.SimpleActionClient('reaching_goal', Planning_ballAction)
     client.wait_for_server()
     goal = Planning_ballGoal()
     time.sleep(5)
     while True:
	 rospy.loginfo(ballDessapear)
	 if ballDessapear == 3:
		# after 4 iteration the ball will gone 
		goal.target_pose.pose.position.z = -1
		client.send_goal(goal)
         	client.wait_for_result()
		time.sleep(10)
   		ballDessapear = 0
         goal.target_pose.pose.position.x = random.randint(-5,5)
         goal.target_pose.pose.position.y = random.randint(-5,5)
	 goal.target_pose.pose.position.z = 0.5
	 client.send_goal(goal)
         client.wait_for_result()
	 randomTime()
	 ballDessapear = ballDessapear + 1

     rospy.spin()
     sis.stop()
	

if __name__ == '__main__':
    main()
