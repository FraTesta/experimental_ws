#!/usr/bin/env python

## @file Rooms.py
#  This script contains the Rooms class, which implements a structure dedicated to the control and management of the rooms in a given environment. 
# It was specially designed for the environment of the final experimental assignment and implemented in the commandManager script.
import actionlib
import rospy 
import random

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

## This class represent the Knowledge representation of the project. It provides a structure (self.ROOMS) that associate each room with a color ball,
# a location (in terms of x and y) and if it's already detected or not.
# It also provides some methods to handle such structure data and exploits it to generate random positions useful for the commandManager.
# Finally it contains the implementation of the exploration algorithm used in the Find state of the commandManager. 
class Rooms():

    def __init__(self):
	##The constructure initialize the ROOMS structure which contains all rooms with the corresponding colors. 
        ## But of course are all to be discovered except the house.
	# @var ROOMS is a standard python dictionary array which contains all corresponding rooms: name, color, position and if they are already visited
	
        self.ROOMS = [ 
        {'name':"Entrance",'color': "blue", "x":0, "y":0, 'detected':False},
        {'name':"Closet",'color': "red", "x":0, "y":0, 'detected':False},
        {'name':"Leavingroom",'color': "green", "x":0, "y":0, 'detected':False},
        {'name':"Kitchen",'color': "yellow", "x":0, "y":0, 'detected':False},
        {'name':"Bathroom",'color': "magenta", "x":0, "y":0, 'detected':False},
        {'name':"Bedroom",'color':"black","x":0,"y":0, 'detected':False},
        {'name':"Home",'color':"","x": -5,"y":7, 'detected':True}
        ]
        self.previousPosX = 0
        self.previousPosY = 0
        self.visitedLocation = [[-5,7]]

    ##Convert a cell array passed into a string
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

    ## Check if a given room name is already visited, if so it returns its position
    # @param target_room name of the desired room 
    def get_room_position(self, target_room):                
        for room in self.ROOMS:
            if target_room == room['name']:
                if room['detected'] == True :                    
                    return [room["x"], room["y"]]        
        return False
    
    ## Returns a list containing all the names of the already visited rooms.
    def visitedList(self):
        vList = []
        for room in self.ROOMS:
            if room['detected'] == True:
                vList.append(room['name'])
        return vList

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
    	return False



    ## Returns a array which contains a neighborhood of a given number. For instance if a = 1 and l = 2 it'll return [-1, 0, 1, 2, 3]
    # @param a number that correspond to a coordinate of a point.
    # @param l is half of the neighborhood that will be generated. 
    def mrange(self, a, l):
        minA = a - l
        r = []
        for i in range(0,l*2+1):
            r.append(minA + i)
        return r
    
    ## Generates a random position in terms of x and y. Usefull in the NORMAL and FIND states
    def generate_rand_pos(self):
        while True:
            tempX = random.randint(-6,6)
            tempY = random.randint(-8,5)
            # generate a position which is different from the previous one
            if tempX != self.previousPosX and tempY != self.previousPosY:  
                # consider the offlimit zone of the environment to simplify the navigation
                if not (3 > tempX > 0 and tempY > 0) and not(tempX < 0 and tempY < -5):
		    self.previousPosX = tempX
		    self.previousPosY = tempY
                    return [tempX, tempY]
                
    ## Cancel the last location added to the visitedLocation list. Necessary when the robot abort a goal position when it sees a new ball  
    def cancel_last_visited_location(self):
        self.visitedLocation.pop()
        

    ## Explore function that returns a random position away from the rooms already visited. Basically it generate a random position and check if it belongs 
    # in the neighborhood of each detected room. 
    def explore(self):
        while True:
            ok = True
            pos = self.generate_rand_pos()
            # check the position is far from the already visited location
            for vis in self.visitedLocation:
                if (pos[0] in self.mrange(vis[0],1)) and (pos[1] in self.mrange(vis[1],1)):
                    #print("[ROOM] Close to a prevoius location ", vis)                    
                    ok = False
                    break
            if ok:
                # check if such position is close or not a already visited room 
                for room in self.ROOMS:
                    if room['detected'] == True:
                        rx = self.mrange(room['x'], 2)
                        ry = self.mrange(room['y'], 2)
                        if (pos[0] in rx and pos[1] in ry):
                            #print("[ROOM] close to a room")
                            ok = False
                            break
            if ok:
                self.visitedLocation.append(pos)
                return pos 
        

        






    


