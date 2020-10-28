#!/usr/bin/env python

from __future__ import print_function

import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from assignment1.srv import *

# service function
def navigation(x,y):

    rospy.wait_for_service('myNavigation')
    try:
        go_to = rospy.ServiceProxy('myNavigation',GoTo)
        check = go_to(x ,y)
        return check.o
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
 

# subsriber call back

def callbackSta(data): 
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def callbackPos(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard x: %d  y: %d", data.linear.x, data.linear.y)

# subscriber manager    
def commandManager():

# node init 
    rospy.init_node('commandManager', anonymous=True) 

    rospy.Subscriber("StateString", String, callbackSta)

    rospy.Subscriber("Position", Twist, callbackPos)

    rate = rospy.Rate(5)

    x = 3
    y = 6

    loop = 1
    while loop ==1:

        navigation(x,y)
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    commandManager()
