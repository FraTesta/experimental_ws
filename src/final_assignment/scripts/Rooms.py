#!/usr/bin/env python
import actionlib
import rospy 
import random

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

    ## Convert a cell array into a string 
    def to_string(self, room):
        delimitat = ""
	return delimitat.join(room)

    def check_visted(self, color):
        for room in self.ROOMS:
            if color == room['color']:
		if room['detected'] == True:
		     return True
        return False
    ## Check if the room contained in msg is already visited, if so then the robot will move to that position 
    def get_room_position(self, target_room):                
        for room in self.ROOMS:
            if target_room == room['name']:
                if room['detected'] == True :
                    
                    return [room["x"], room["y"]]
        
        return False

    def add_new_room(self, color, x, y):
        for room in self.ROOMS:
            if color == room['color']:
                room['detected'] = True
                room['x'] = int(x)
                room['y'] = int(y)
		name = self.to_string(room['name'])
		print("[Rooms] %s Discovered and added correctly: ", name )

    def get_name_position(self, x, y):
        for room in self.ROOMS:
            if (x == room['x'] and y == room['y']):
                return room['name']
	return False

    def generate_rand_pos(self):
        while True:
            tempX = random.randint(-5,5)
            tempY = random.randint(-5,5)
            if not (tempX > 0 and tempY > 3):
                    return [tempX, tempY]

    def mrange(self, a):
        minA = a - 3
        r = []
        for i in range(0,7):
            r.append(minA + i)
        return r

    def explore(self):
        while True:
            ok = True
            pos = self.generate_rand_pos()
            for room in self.ROOMS:
                if room['detected'] == True:
                    rx = self.mrange(room['x'])
                    ry = self.mrange(room['y'])
                    if (pos[0] in rx and pos[1] in ry):
                        ok = False
            if ok:
                return pos 
        

        






    


