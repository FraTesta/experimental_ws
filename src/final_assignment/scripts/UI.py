#!/usr/bin/env python  
import rospy 
from std_msgs.msg import String
import time

def UI():
    rospy.init_node('UI', anonymous=True)
    pub = rospy.Publisher('UIchatter', String, queue_size=10) 
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
	msg = raw_input("Type 'Play' to switch in the PLAY mode:")
	if msg == 'Play' or msg == 'play':
        	pub.publish(msg)
        	rate.sleep()
        	msg = raw_input("Please enter the room that you want to go (GoTo roomName): ")
        	rospy.loginfo("wait ...")
        	pub.publish(msg)
        	time.sleep(5)
    else:
        rospy.logerr("[Syntax Error] please enter 'play' or 'Play'")
	
	
            
	

if __name__ == "__main__":
    try:
        UI()
	
    except rospy.ROSInterruptException:
        rospy.loginfo(" UI process termiated !")
