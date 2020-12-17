#!/usr/bin/env python
import rospy 
import random
import actionlib
import actionlib_tutorials.msg
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkState
import assignment2.msg 





def randomTime():
	 rate = rospy.Rate(1)
	 randi = random.randint(1,6)
	 for i in range(randi):
		rate.sleep()


def main():
	
	client = actionlib.SimpleActionClient('reaching_goal', assignment2.msg.Planning_ballAction)
	client.wait_for_server()

	goal = Planning_BallGoal()

	while True:
            goal.target_pose.pose.position.x = random.randint(-7,7)
            goal.target_pose.pose.position.y = random.randint(-7,7)
	    rospy.loginfo("I'm going to position x = %d y = %d", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
	    client.send_goal(goal)
            client.wait_for_result()
	    randomTime()
	

if __name__ == '__main__':
    main()
