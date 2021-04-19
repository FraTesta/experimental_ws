#!/usr/bin/env python

## @file commandManager.py
#  This script is a node which is the core of the entier project.
#  And implement a finite state machine with five states which are described in the README file.
#  It manages the messages comming from other nodes and ROS packeges.
# \see roomDetector
# \see UI
# \see track
 

from __future__ import print_function

import roslib
import rospy
import smach
import smach_ros
import time
import random
import sys

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Bool
from Rooms import Rooms

import smach_msgs.msg 


# Action Server 
import actionlib
import actionlib.msg
from final_assignment.msg import trackBallGoal, trackBallAction

# Flags and global variables 

## Flag which notify when the user types 'play'
PLAY = False
## Contains the desired room entered by the user 
TARGET_ROOM = "None"
## Flag which notify when the command manager recives a new target room from the user
NEW_TR = False
## Flag which notify that a new room was detected by the camera
NEW_ROOM = False
## Contains the color of the detected ball (room) 
COLOR_ROOM = "None"
## Flag indicating if the FIND mode is active or not 
FIND_MODE = False
## Initialization of the move_base client in order to assign target position to the move_base action server
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
## Publisher to the startRD topic which allows to enable/disable the room detector 
roomD_pub = rospy.Publisher('startRD', Bool, queue_size=10)
## Object of the class Rooms necessary to store and manage the discovered rooms 
rooms = Rooms()




## Callback mathod of the UIsubscriber which handels the commands sent by the user
# @param data is a string message coming from the UI ROS node
def UIcallback(data):
    global PLAY, TARGET_ROOM, NEW_TR, rooms , client, roomD_pub
    if data.data == "play" or data.data == "Play":
        rospy.loginfo("[CommandManager] Recived a 'play' request!")
	PLAY = True
        client.cancel_all_goals()
	# stop detecting
	roomD_pub.publish(False)
	time.sleep(3)
        

    elif data.data.startswith("GoTo"):
        NEW_TR = True
        TARGET_ROOM = data.data
        rospy.loginfo("[CommandManager] I recived the desired room whose name is: %s", TARGET_ROOM)
    else:
	rospy.logerr("[Syntax Error] the sent msg is wrong")

## Callback function of the newRoomSub subscriber which recives the color of the new detected room
# @param color is  a string message containing the color of the room detected by the roomDetector ROS node
def newRoomDetected(color):
    global NEW_ROOM, COLOR_ROOM, client, rooms, roomD_pub
        
    if not rooms.check_visted(color.data):
	rospy.loginfo("[CommandManager] reach a new ball of color %s", color.data)
	NEW_ROOM = True
	# stop detecting 
	roomD_pub.publish(False)
	COLOR_ROOM = color.data
    	client.cancel_all_goals()

## Method which prepares and send the goal to the move_base action server 
# @param x X position of the desired goal 
# @param y Y position of the desired goal 
def move_base_go_to(x, y):
    global client, PLAY, NEW_ROOM 
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = 0
    goal.target_pose.pose.orientation.w = 2.0

    rospy.loginfo("[CommandManager] I'm going to position x = %d y = %d", x, y)
    
    client.send_goal(goal)
    wait = client.wait_for_result()
    
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
	name = rooms.get_name_position(x, y)
	if PLAY or NEW_ROOM:
	      rospy.loginfo("[CommandManager] MoveBase mission aborted!!!")
	elif not name:
              rospy.loginfo("[CommandManager] Position (%d,%d) reached. wait...", x, y)
	else:
              rospy.loginfo("[CommandManager] %s reached. wait...", name) 
	time.sleep(4)


 

class Normal(smach.State):
    '''This class defines the NORMAL state of the FSM. In particular It sends random position to the move_base action server.
    It checks whether a ball is detected in order to switch in the TRACK state and check if the user wants to switch in the PLAY state as well.
    Otherwise after some iterations it goes in SLEEP mode 
    '''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep','goToPlay','goToTrack'])

	# Loop at 200 Hz
        self.rate = rospy.Rate(1)  
	## Counter variable to check the number of iteration of the NORMAL state in order to move to SLEEP state after a certain number 
        self.counter = 0
        
    def execute(self,userdata):
	""" Method inherited from the smach.State class """
        global PLAY, client, NEW_ROOM, rooms, roomD_pub 
	rospy.loginfo("***********************************")
	rospy.loginfo("[CommandManager] I m in NORMAL state")
	time.sleep(3)
	#Start the roomDetector
	roomD_pub.publish(True)

	self.counter = 0
        
        while not rospy.is_shutdown():  

            if PLAY == True:
                return 'goToPlay'
            elif self.counter == 4:
                return 'goToSleep' 
            elif NEW_ROOM == True:
                return 'goToTrack'
		
	    else:

                # move in a random position using move_base
	        rospy.loginfo("[CommandManager] generate a new random goal position")
                pos = rooms.generate_rand_pos()
		move_base_go_to(pos[0], pos[1])
                self.rate.sleep()		
                self.counter += 1
             
        
    


class Sleep(smach.State):
    '''It defines the SLEEP state where the robot sleeps for a random period of time.
    Then it makes a request to the move_base action server to go to the home location.
    Finally it returns in the NORMAL state'''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep']) 
                             
        self.rate = rospy.Rate(200)  # Loop at 200 Hz

    def execute(self, userdata): 
	global rooms
	rospy.loginfo("***********************************")
        rospy.loginfo("[CommandManager] I m in SLEEP mode")
        # comeback to home      
        position = rooms.get_room_position("Home")
	move_base_go_to(position[0], position[1])     
	rospy.loginfo("[CommandManager] Home reached")
        # sleep for a random time period
	rospy.loginfo("[CommandManager] Sleeping...")
        time.sleep(random.randint(3,6))
        return 'goToNormal'


class Play(smach.State):
    '''Class that defines the PLAY state. 
     It move the robot in X Y location and then asks to go back to the user.'''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToPlay','goToFind'])

        self.rate = rospy.Rate(200)
	## Variable to count the time pass
        self.counter = 0
	    

    def execute(self, userdata):
	rospy.loginfo("***********************************")
        rospy.logdebug("[CommandManager] I m in PLAY mode")
	global PLAY, rooms 
	time.sleep(3)

        # go to the person
        position = rooms.get_room_position("Home")
        move_base_go_to(position[0], position[1])	       

        while not rospy.is_shutdown():
	    # we need to update them at each iteration 
	    global TARGET_ROOM, NEW_TR
	    PLAY = False

            if self.counter <= 5:
                if NEW_TR == True:
                                        
                    if TARGET_ROOM.startswith("GoTo"):
                        TARGET_ROOM = TARGET_ROOM.strip("GoTo ")
                        position = rooms.get_room_position(TARGET_ROOM) 
                        if not position:
                            rospy.loginfo("[CommandManager] That room has not yet been visited")
			    return 'goToFind'  
                        else:
			    rospy.loginfo("[CommandManager] Room already visited!")
                            move_base_go_to(position[0], position[1])                 
                    else:
                        rospy.logerr("[Sintax Error] no GoTo command typed!") 
                    time.sleep(1)
		    rospy.loginfo("[CommandManager] Homecoming...")
                    # comebak to the person 
                    position = rooms.get_room_position("Home")
                    move_base_go_to(position[0], position[1])
		    NEW_TR = False
                
	        # wait a few seconds 
                time.sleep(3)
		rospy.loginfo("[CommandManager] Await location-..........")
               	self.counter += 1
                
	    else:
		self.counter = 0
            	return 'goToNormal'

class Track(smach.State):
    '''Class that defines the TRACK state. 
     It send the detected color room to the track action server in order ot reach it. 
     Then sotores the location of the room found '''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToPlay','goToFind','goToTrack'])

    def execute(self, userdata):
        global rooms, NEW_ROOM, COLOR_ROOM, NEW_TR, FIND_MODE
	rospy.loginfo("***********************************")
	rospy.loginfo("[CommandManager] I'm in TRACK state")
        goal = trackBallGoal()
        goal.color = COLOR_ROOM

        trackClient = actionlib.SimpleActionClient('trackAction',trackBallAction)
        trackClient.wait_for_server()
	NEW_ROOM = False
        trackClient.send_goal(goal)
        wait = trackClient.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return "goToNormal"
        else:
            
            result = trackClient.get_result()
            # Since if the result is (0,0) means that no ball has been reached
	    if result.x != 0 and result.y != 0:
	          rospy.loginfo("[CommandManager] New Room reached!")
		  # add the the room to the structure
	          rooms.add_new_room(COLOR_ROOM, result.x, result.y)
		  # check which is the next state
                  if FIND_MODE == True:
                  	if COLOR_ROOM == rooms.get_color_room(TARGET_ROOM):
		      	    FIND_MODE = False
			    rospy.loginfo("Desired room discovered!!!")
                            return 'goToPlay'		      
                  	else:
			    rospy.loginfo("The room just found isn't the desired one")
                            return 'goToFind'
	    else:
		 rospy.loginfo("[CommandManager] The robot is not able to find the previously detected ball")
            
            return "goToNormal"

class Find(smach.State):
    '''Class that defines the FIND state. 
     It move the robot using the explore function in order to find the esired room or a new one.'''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToPlay','goToTrack','goToFind'])
	self.rate = rospy.Rate(1)  # Loop at 200 Hz

    def execute(self, userdata):
        global rooms, NEW_ROOM, COLOR_ROOM, FIND_MODE
        rospy.loginfo("***********************************")
        rospy.loginfo("[CommandManager] I'm in FIND state")
        FIND_MODE = True
	roomD_pub.publish(True)
        time.sleep(4)
        self.counter = 0
        
        while not rospy.is_shutdown():  

            if self.counter == 4:
		FIND_MODE = False
		roomD_pub.publish(False)
                return 'goToPlay' 
            elif NEW_ROOM == True:
                return 'goToTrack'
		
	    else:
	        rospy.loginfo("[CommandManager] Exploring....")
                pos = rooms.explore()
		move_base_go_to(pos[0], pos[1])
                self.rate.sleep()
                self.counter += 1
        
def main():
    
    global client
    try:
        rospy.init_node('commandManager')
        # initialize the move_base client
        client.wait_for_server()
        ## Subscriber to the UIchatter topic
        UIsubscriber = rospy.Subscriber("UIchatter", String, UIcallback)
        ## Subscriber to the newRoom topic 
        newRoomSub = rospy.Subscriber("newRoom", String, newRoomDetected)
        
        
        ## Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['init'])
    

        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add('NORMAL', Normal(), 
                                transitions={'goToSleep':'SLEEP', 
                                                'goToPlay':'PLAY',
						'goToTrack' : 'TRACK',
                                                'goToNormal':'NORMAL'})
            smach.StateMachine.add('SLEEP', Sleep(), 
                                transitions={'goToSleep':'SLEEP', 
                                                'goToNormal':'NORMAL'})
            smach.StateMachine.add('PLAY', Play(), 
                                transitions={'goToNormal':'NORMAL',
					      'goToFind':'FIND',
                                                'goToPlay':'PLAY'})
            smach.StateMachine.add('TRACK', Track(), 
                                transitions={'goToNormal':'NORMAL',
                                                'goToPlay':'PLAY',
                                                'goToFind':'FIND',
                                                'goToTrack':'TRACK'})
            smach.StateMachine.add('FIND', Find(), 
                                transitions={
                                                'goToTrack':'TRACK',
                                                'goToPlay':'PLAY',
                                                'goToFind':'FIND'})


        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()

        outcome = sm.execute()

        rospy.spin()
        sis.stop()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")


if __name__ == '__main__':
    main()
