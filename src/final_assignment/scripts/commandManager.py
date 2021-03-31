#!/usr/bin/env python
import actionlib
import rospy 

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from Rooms import Rooms

# initialize the environment 
rooms = Rooms()
rooms.ROOMS[1]['detected'] = True
   

def UIcallback(data):
    global rooms
    msg = data.data
    print("I recived %s", msg)

    if msg.startswith("GoTo"):
         target_room = msg.strip("GoTo ")
         if not rooms.check_visited(target_room):
             print("that room has not yet been visited")

    else:
         rospy.logerr("[Sintax Error] no GoTo command typed!")
          

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        UIsubscriber = rospy.Subscriber("UIchatter", String, UIcallback)
    	rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
