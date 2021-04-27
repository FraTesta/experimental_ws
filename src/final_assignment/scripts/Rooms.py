#!/usr/bin/env python

## @file Rooms.py
#  This script contains the Rooms class, which implements a structure dedicated to the control and management of the rooms in a given environment. 
# It was specially designed for the environment of the final experimental assignment and implemented in the commandManager script.
import actionlib
import rospy 
import random

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Rooms():
    """ Class provides a smart structure to store the discoverd rooms, and some methods to handle it

    """

    def __init__(self):
	##The constructure initialize the ROOMS structure which contains all rooms with the corresponding colors. 
        ## But of course are all to be discoverd except the house.
	# @var ROOMS is a standard python dictionary array which contains all corresponding rooms: name, color, position and if they are already visited
	
        self.ROOMS = [ 
        {'name':"Entrance",'color': "blue", "x":0, "y":0, 'detected':False},
        {'name':"Closet",'color': "red", "x":0, "y":0, 'detected':False},
        {'name':"LeavingRoom",'color': "green", "x":0, "y":0, 'detected':False},
        {'name':"Kitchen",'color': "yellow", "x":0, "y":0, 'detected':False},
        {'name':"BathRoom",'color': "magenta", "x":0, "y":0, 'detected':False},
        {'name':"BedRoom",'color':"black","x":0,"y":0, 'detected':False},
        {'name':"Home",'color':"","x": -5,"y":7, 'detected':True}
        ]

    ##Convert a cell array returned by the ROOMS structure into a string
    #@param room is a generic room cell of the ROOMS dictionary   
    def to_string(self, room):
        delimitat = ""
	return delimitat.join(room)

    ## Checks if a room of a given color is already visited.
    # @param color color of the room to be checked
    def check_visted(self, color):
        for room in self.ROOMS:
            if color == room['color']:
		if room['detected'] == True:
		     return True
        return False

    ## Check if a given room is already visited, if so it returns its position
    # @param target_room name of the room 
    def get_room_position(self, target_room):                
        for room in self.ROOMS:
            if target_room == room['name']:
                if room['detected'] == True :                    
                    return [room["x"], room["y"]]        
        return False

    ## Adds a new room to the ROOMS structure 
    # @param color color of the discovered room 
    # @param x x position of the discovered room
    # @param y y position of the discovered room
    def add_new_room(self, color, x, y):
        for room in self.ROOMS:
            if color == room['color']:
                room['detected'] = True
                room['x'] = int(x)
                room['y'] = int(y)
		name = self.to_string(room['name'])
		print("[Rooms] Discovered and added correctly: " + name )

    ## Returns the name of a room located in a given position (if any)
    # @param x x position of the room
    # @param y y position of the room
    def get_name_position(self, x, y):
        for room in self.ROOMS:
            if (x == room['x'] and y == room['y']):
                return room['name']
	return False

    ## Returns the color of a room given its name
    # @param name room name 
    def get_color_room(self, name):
	for room in self.ROOMS:
	    if name == room['name']:
		return room['color']

    ## Generates a random position in terms of x and y. Usefull in the NORMAL and FIND states
    def generate_rand_pos(self):
        while True:
            tempX = random.randint(-5,5)
            tempY = random.randint(-8,5)
            if not (3 > tempX > 0 and tempY > 0) and not(tempX < 0 and tempY < -5):
                    return [tempX, tempY]

    ## Returns a array which contains a neighborhood of a given number
    # @param number that correspond to a coordinate of a point
    def mrange(self, a):
        minA = a - 3
        r = []
        for i in range(0,7):
            r.append(minA + i)
        return r

    ## Returns a random position away from the rooms already visited
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
        

        






    


