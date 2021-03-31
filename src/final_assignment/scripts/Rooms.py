#!/usr/bin/env python
import actionlib
import rospy 

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Rooms():

    def __init__(self):
        self.ROOMS = [ 
        {'name':"Entrance",'colour': "blue", "x":0, "y":0, 'detected':False},
        {'name':"Closet",'colour': "red", "x":0, "y":0, 'detected':False},
        {'name':"LeavingRoom",'colour': "green", "x":0, "y":0, 'detected':False},
        {'name':"Kitchen",'colour': "yellow", "x":0, "y":0, 'detected':False},
        {'name':"BathRoom",'colour': "orange", "x":0, "y":0, 'detected':False},
        {'name':"BedRoom",'colour':"black","x":0,"y":0, 'detected':False}
        ]

    
# check if the room contained in msg is already visited, if so then the robot will move to that position 
    def check_visited(self, target_room):
        #print(self.ROOMS)
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        for room in self.ROOMS:
            if target_room == room['name']:
                if room['detected'] == True :
                    print("Such room is already visited!!!")
                    # go to the room position using the move_base action server
                    goal.target_pose.pose.position.x = room["x"]
                    goal.target_pose.pose.position.y = room["y"]
                    goal.target_pose.pose.orientation.w = 1.0
                    # send goal to move_base 
                    client.send_goal(goal)
                    wait = client.wait_for_result()
                    if not wait:
                        rospy.logerr("Action server not available!")
                        rospy.signal_shutdown("Action server not available!")
                    else:
                        rospy.loginfo("Goal execution done!!!!")

                    return True
        
        return False

