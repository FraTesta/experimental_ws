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
    def get_room_position(self, target_room):
        
        
        for room in self.ROOMS:
            if target_room == room['name']:
                if room['detected'] == True :
                    print("Such room is already visited!!!")


                    return [room["x"], room["y"]]
        
        return False


    def add_new_room(self, color, x, y):
        for room in self.ROOMS:
            if color == room['color']:
                room['detected'] = True
                room['x'] = x
                room['y'] = y

    def go_to_position(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0
        # send goal to move_base 
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return False
        else:
            rospy.loginfo("Room reached!!!!")
            return True

    def stop_moving(self):
        self.client.cancel_all_goals()


    


