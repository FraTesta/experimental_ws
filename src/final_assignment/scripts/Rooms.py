#!/usr/bin/env python
import actionlib
import rospy 

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Rooms():

    def __init__(self):
        self.ROOMS = [ 
        {'name':"Entrance",'color': "blue", "x":0, "y":0, 'detected':False},
        {'name':"Closet",'color': "red", "x":0, "y":0, 'detected':False},
        {'name':"LeavingRoom",'color': "green", "x":0, "y":0, 'detected':False},
        {'name':"Kitchen",'color': "yellow", "x":0, "y":0, 'detected':False},
        {'name':"BathRoom",'color': "orange", "x":0, "y":0, 'detected':False},
        {'name':"BedRoom",'color':"black","x":0,"y":0, 'detected':False},
        {'name':"Home",'color':"","x":-5,"y":7, 'detected':True}
        ]

    
# check if the room contained in msg is already visited, if so then the robot will move to that position 
    def go_to_room(self, target_room):
        #print(self.ROOMS)
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        for room in self.ROOMS:
            if target_room == room['name']:
                if room['detected'] == True :
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
                        rospy.loginfo("Room reached: ",room['name'] )

                    return True
        
        return False


    def add_new_room(self, color, x, y):
        for room in self.ROOMS:
            if color == room['color']:
                room['detected'] = True
                room['x'] = x
                room['y'] = y
    


