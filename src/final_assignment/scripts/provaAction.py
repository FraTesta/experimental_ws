#!/usr/bin/env python
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Point, Pose

# Action Server 
import actionlib
import actionlib.msg
import final_assignment.msg


def main():
    rospy.init_node('provaAction')

    goal = final_assignment.msg.trackBallGoal()

    goal.color = "blue"
    client = actionlib.SimpleActionClient('robot_reach_room', final_assignment.msg.trackBallAction)
    rospy.loginfo("client created ")
    while not rospy.is_shutdown():  
        client.send_goal(goal)
	rospy.loginfo("goal sent")
        client.wait_for_result()
	rospy.loginfo("goal recived")
        result = client.get_result()
        rospy.loginfo("la posizione attuale e':")
        rospy.loginfo(result.x)
        rospy.loginfo(result.y)

if __name__ == "__main__":
    main()
