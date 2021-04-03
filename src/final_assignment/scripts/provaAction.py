#!/usr/bin/env python
rom __future__ import print_function

import roslib
import rospy
import smach
import smach_ros
import time
import random
import sys
import random 

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from Rooms import Rooms

import smach_msgs.msg 

client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
client.wait_for_server()


# Action Server 
import actionlib
import actionlib.msg
from final_assignment.msg import trackBallGoal, trackBallAction
def move_base_go_to(x, y):
    global client
    ## init move_base goal 
    goal = MoveBaseGoal()
    ## set the goal as a random position 
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("I'm going to position x = %d y = %d", x, y)
    
    client.send_goal(goal)
    wait = client.wait_for_result()
    
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Goal reached!!!!")

def main():
    time.sleep(6)
    move_base_go_to(-1,-1)
    time.sleep(3)
    move_base_go_to(-1, 3)

if __name__ == '__main__':
    main()
