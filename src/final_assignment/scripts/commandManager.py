#!/usr/bin/env python

## @file commandManager.py
#  This node includes the subsription to State and GetPosition publishers,
#  And implement a finite state machine 
#  which manages the information coming from the two publisher and changes the state of the system in according to them.
# \see getPosition.cpp
# \see Navigation.cpp
# \see State.cpp
 

from __future__ import print_function

import roslib
import rospy
import smach
import smach_ros
import time
import random
import sys
import random 

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
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
## Flag which notify when the command manager recives a new target room
NEW_TR = False
## new room detected 
NEW_ROOM = False
## Random position generation flag
RANDOM = True 
## color of the detected ball 
COLOR_ROOM = "None"

## init the move_base client 
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

## init the rooms environment 
rooms = Rooms()





def UIcallback(data):
    global PLAY, TARGET_ROOM, NEW_TR, rooms , client
    if data.data == "play" or data.data == "Play":
        rospy.loginfo("[CommandManager] Recived a 'play' request!")
        client.cancel_all_goals()
	time.sleep(3)
        PLAY = True

    elif data.data.startswith("GoTo"):
        NEW_TR = True
        TARGET_ROOM = data.data
        rospy.loginfo("[CommandManager] I recived the desired room whose color is: %s", TARGET_ROOM)
    else:
	rospy.logerr("[Syntax Error] the sent msg is wrong")


def newRoomDetected(color):
    global NEW_ROOM, COLOR_ROOM, client
    rospy.loginfo("[CommandManager] reach a new ball of color %s, start track it !", color.data)
    NEW_ROOM = True 
    COLOR_ROOM = color.data
    client.cancel_all_goals()


def move_base_go_to(x, y):
    global client
    ## init move_base goal 
    goal = MoveBaseGoal()
    ## set the goal as a random position 
    goal.target_pose.header.frame_id = "odom"
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
        rospy.loginfo("[CommandManager] [Move Base] Action server closed. wait...")
	time.sleep(4)

def generate_rand_pos():
    while True:
         tempX = random.randint(-5,5)
         tempY = random.randint(-5,5)
	 if not (tempX > 0 and tempY > 3):
                return [tempX, tempY]

    

 

class Normal(smach.State):
    '''This class defines the NORMAL state of the FSM. In particular It sends random position to the navigation_action server
    and it checks whether a ball is detected in order to move to PLAY state.
    Otherwise after some iterations it goes in SLEEP mode '''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep','goToPlay','goToTrack'])


        self.rate = rospy.Rate(1)  # Loop at 200 Hz
	## Counter variable to check the number of iteration of the NORMAL state in order to move to SLEEP state after a certain number 
        self.counter = 0
        
    def execute(self,userdata):
        global PLAY, client, NEW_ROOM
	rospy.loginfo("***********************************")
	rospy.loginfo("[CommandManager] I m in NORMAL state")
	time.sleep(4)
	self.counter = 0
        
        while not rospy.is_shutdown():  

            if PLAY == True:
                return 'goToPlay'
            elif self.counter == 4:
                return 'goToSleep' 
            elif NEW_ROOM == True:
                return 'goToTrack'
		
	    else:
	        #move_base_go_to(-2, 8)

                # move in a random position using move_base
	        rospy.loginfo("[CommandManager] generate a new random goal position")
                #move_base_go_to(random.randint(-5,5), random.randint(-5,5))
                pos = generate_rand_pos()
		move_base_go_to(pos[0], pos[1])
                self.rate.sleep()
		
                self.counter += 1
            
        return 'goToSleep' 
        
    


class Sleep(smach.State):
    '''It defines the SLEEP state which sleeps for a random period of time.
    Then it makes a request to the Navigation service to go to the home location.
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
                             outcomes=['goToNormal','goToPlay'])

        self.rate = rospy.Rate(200)
        self.counter = 0
	    

    def execute(self, userdata):
	rospy.loginfo("***********************************")
        rospy.logdebug("[CommandManager] I m in PLAY mode")
	global PLAY, rooms 

        

        # go to the person
        #userdata.rooms_in.go_to_room("Home")
        position = rooms.get_room_position("Home")
        # go to Home position 
        move_base_go_to(position[0], position[1])	
	rospy.loginfo("[CommandManager] Home Reached!!!!")
        

        while not rospy.is_shutdown():
	    # we need to update them at each iteration 
	    global TARGET_ROOM, NEW_TR

            if self.counter <= 5:
                if NEW_TR == True:
                                        
                    if TARGET_ROOM.startswith("GoTo"):
                        TARGET_ROOM = TARGET_ROOM.strip("GoTo ")

                        position = rooms.get_room_position(TARGET_ROOM)
                        
                        if not position:
                            rospy.loginfo("[CommandManager] That room has not yet been visited")  
                        else:
                            move_base_go_to(position[0], position[1])                 

                    else:
                        rospy.logerr("[Sintax Error] no GoTo command typed!")

                    # wait a few seconds 
                    time.sleep(1)
		    rospy.loginfo("[CommandManager] Homecoming...")
                    # comebak to the person 
                    position = rooms.get_room_position("Home")
                    move_base_go_to(position[0], position[1])
		    rospy.loginfo("[CommandManager] Home Reached!!!!")
		    NEW_TR = False
                
	        # wait a few seconds 
                time.sleep(3)
		rospy.loginfo("[CommandManager] wait-..........")
               	self.counter += 1
                
	    else:
		self.counter = 0
            	PLAY = False
            	return 'goToNormal'

class Track(smach.State):
    '''Class that defines the PLAY state. 
     It move the robot in X Y location and then asks to go back to the user.'''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToTrack'])

    def execute(self, userdata):
        global rooms, NEW_ROOM, COLOR_ROOM
	rospy.loginfo("***********************************")
	rospy.loginfo("[CommandManager] I'm in TRACK state")
        goal = trackBallGoal()
        goal.color = COLOR_ROOM

        trackClient = actionlib.SimpleActionClient('trackAction',trackBallAction)
        trackClient.wait_for_server()
        rospy.loginfo("Track client creato")

        trackClient.send_goal(goal)
        wait = trackClient.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            NEW_ROOM = False
            return "goToNormal"
        else:
            rospy.loginfo("[CommandManager] New Room reached!")
            result = trackClient.get_result()
            rospy.loginfo("[CommandManager] la posizione attuale e':")
            rospy.loginfo(result.x)
            rospy.loginfo(result.y)
	    ###### add the new room to the list ######
            NEW_ROOM = False
            return "goToNormal"
            
       



	

        
def main():
    
    #global client
    try:
        rospy.init_node('commandManager')
        # move_base client
        client.wait_for_server()
        # Subscriber to the UIchatter topic
        UIsubscriber = rospy.Subscriber("UIchatter", String, UIcallback)
        # Subscriber to the newRoom topic 
        newRoomSub = rospy.Subscriber("newRoom", String, newRoomDetected)
        
        
        # Create a SMACH state machine
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
                                                'goToPlay':'PLAY'})
            smach.StateMachine.add('TRACK', Track(), 
                                transitions={'goToNormal':'NORMAL',
                                                'goToTrack':'TRACK'})


        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()

        outcome = sm.execute()

        rospy.spin()
        sis.stop()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")


if __name__ == '__main__':
    main()
