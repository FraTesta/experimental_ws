#!/usr/bin/env python  
import rospy 
from std_msgs.msg import String

def UI():
    rospy.init_node('UI', anonymous=True)
    pub = rospy.Publisher('UIchatter', String, queue_size=10) 
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg = raw_input("Please enter the room that you want to go (GoTo roomName): ")

        rospy.loginfo("the desired room is: %s", msg)
        pub.publish(msg)
        rate.sleep()
        


if __name__ == "__main__":
    try:
        UI()
	
    except rospy.ROSInterruptException:
        rospy.loginfo(" UI process termiated !")
