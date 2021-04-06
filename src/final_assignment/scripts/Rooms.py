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
        {'name':"Home",'color':"","x": -5,"y":7, 'detected':True}
        ]



    
# check if the room contained in msg is already visited, if so then the robot will move to that position 
    def get_room_position(self, target_room):
        
        
        for room in self.ROOMS:
            if target_room == room['name']:
                if room['detected'] == True :
                    print("[Rooms] Such room is already visited!!!")


                    return [room["x"], room["y"]]
        
        return False


    def add_new_room(self, color, x, y):
        for room in self.ROOMS:
            if color == room['color']:
                room['detected'] = True
                room['x'] = (int)x
                room['y'] = (int)y
		name = to_string(room['name'])
		print("[Rooms] %s Discovered and added correctly: ", name )

    def get_name_position(self, x, y):
        for room in self.ROOMS:
            if (x == room['x'] and y == room['x']):
                return room['name']
	return False

    def to_string(room):
        delimitat = ""
	return delimitat.join(room)

    


