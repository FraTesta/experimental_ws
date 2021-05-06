#!/usr/bin/env python 

## @file UI.py
# It's a script that defines the user interface  

import rospy 
from std_msgs.msg import String
import time

## Mthods that runs the user interface
def UI():
    ## Publisher to the commandManger in order to comunicate the commands entered by the user
    pub = rospy.Publisher('UIchatter', String, queue_size=10) 
    rate = rospy.Rate(10)
    print("******************************\n Welcome !!!! \n\n The user can digit the keyword: \n - 'play' -> to switch in play mode \n - 'GoTo roomName' -> to reach that room or to start looking for it (if it hasn't yet been discovered) \n - 'list' -> to get the list of the rooms already visited \n The rooms present are:\n Entrance(blue)\n Closet(red)\n Leavingroom(green)\n Kitchen(yellow)\n Bathroom(Magenta)\n Bedroom(black)\n*******************************\n")

    while not rospy.is_shutdown():
	msg = raw_input("User:")
        pub.publish(msg)
        rate.sleep()
	
	
            
	

if __name__ == "__main__":
    rospy.init_node('UI', anonymous=True)
    try:
        UI()
	
    except rospy.ROSInterruptException:
        rospy.loginfo(" UI process termiated !")
