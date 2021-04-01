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
	

   	pub.publish(msg)
	
	rate.sleep()
        msg = raw_input("Please enter the room that you want to go (GoTo roomName): ")
	rospy.loginfo("wait ...")
        pub.publish(msg)
	time.sleep(3)
            
	 
	
        


if __name__ == "__main__":
    try:
        UI()
	
    except rospy.ROSInterruptException:
        rospy.loginfo(" UI process termiated !")
