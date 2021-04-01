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

# Flag which notify when the user types 'play'
PLAY = False

# Contains the desired room entered by the user 
TARGET_ROOM = "None"

# Flag which notify when the command manager recives a new target room
NEW_TR = False

## init move_base client 
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)



def UIcallback(data):
    global PLAY, TARGET_ROOM, NEW_TR, client
    if data.data == "play" or data.data == "Play":
        rospy.loginfo("Recived a 'play' request...stop every move_base goal!!")
        client.cancel_all_goals()
        PLAY = True

    else:
        NEW_TR = True
        TARGET_ROOM = data.data
        print("I recived the desired room: %s", TARGET_ROOM)


def newRoomDetected(color):
    rospy.loginfo("[CmdMg] reach a new ball, start track it !")
    goal = trackBallGoal()
    goal.color = color.data

    client = actionlib.SimpleActionClient('trackAction',trackBallAction)
    client.wait_for_server()
    rospy.loginfo("Track client creato")

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo("Goal execution done!!!!")
	result = client.get_result()
	rospy.loginfo("la posizione attuale e':")
    	rospy.loginfo(result.x)
    	rospy.loginfo(result.y)


 

class Normal(smach.State):
    '''This class defines the NORMAL state of the FSM. In particular It sends random position to the navigation_action server
    and it checks whether a ball is detected in order to move to PLAY state.
    Otherwise after some iterations it goes in SLEEP mode '''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep','goToPlay'],
                             input_keys=['rooms_in'], # msg in input 
                             output_keys=['rooms_out']) # msg in output)

        ## init move_base goal 
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()

        self.rate = rospy.Rate(1)  # Loop at 200 Hz
	## Counter variable to check the number of iteration of the NORMAL state in order to move to SLEEP state after a certain number 
        self.counter = 0
        
    def execute(self,userdata):
        global PLAY
        
        while not rospy.is_shutdown():  

            if PLAY == True:
                return 'goToPlay'
            if self.counter == 4:
                return 'goToSleep'           
            # move in a random position using move_base




            ## set the goal as a random position 
            self.goal.target_pose.pose.position.x = random.randint(-5,5)
            self.goal.target_pose.pose.position.y = random.randint(-5,5)
	    self.goal.target_pose.pose.orientation.w = 1.0

            rospy.loginfo("I'm going to position x = %d y = %d", self.goal.target_pose.pose.position.x, 	    self.goal.target_pose.pose.position.y)
            client.send_goal(self.goal)
            wait = client.wait_for_result()
            if not wait:
                rospy.logerr("Action server not available!")
                rospy.signal_shutdown("Action server not available!")
            else:
                rospy.loginfo("Goal execution done!!!!")

	        self.rate.sleep()
            self.counter += 1
            
        return 'goToSleep' 
        
    


class Sleep(smach.State):
    '''It defines the SLEEP state which sleeps for a random period of time.
    Then it makes a request to the Navigation service to go to the home location.
    Finally it returns in the NORMAL state'''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep'],
                             input_keys=['rooms_in'], # msg in input 
                             output_keys=['rooms_out']) # msg in output)
                             
        self.rate = rospy.Rate(200)  # Loop at 200 Hz

    def execute(self, userdata): 
        rospy.loginfo("I m in SLEEP mode")
        # comeback to home      
        userdata.rooms_in.go_to_room("Home")
     
	rospy.loginfo("Home reached")
        # sleep for a random time period
        time.sleep(random.randint(3,6))
#        self.rate.sleep()
        userdata.rooms_out = userdata.rooms_in
        return 'goToNormal'


class Play(smach.State):
    '''Class that defines the PLAY state. 
     It move the robot in X Y location and then asks to go back to the user.'''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToPlay'],
                             input_keys=['rooms_in'], # msg in input 
                             output_keys=['rooms_out']) # msg in output))
        
        self.rate = rospy.Rate(200)
        self.counter = 0
	    

    def execute(self, userdata):

        rospy.loginfo("I m in PLAY mode")
	global TARGET_ROOM, PLAY, NEW_TR

        rospy.loginfo(userdata.rooms_in.ROOMS[0]['name'])

        # go to the person
        userdata.rooms_in.go_to_room("Home")

        while not rospy.is_shutdown():

            if self.counter <= 3:
                if NEW_TR == True:
                                        
                    if msg.startswith("GoTo"):
                        TARGET_ROOM = msg.strip("GoTo ")
                        
                        if not userdata.rooms_in.go_to_room(TARGET_ROOM):
                            print("that room has not yet been visited")                   

                    else:
                        rospy.logerr("[Sintax Error] no GoTo command typed!")

                    # wait a few seconds 
                    time.sleep(3)
                    # comebak to the person 
                    userdata.rooms_in.go_to_room("Home")
                

                rate.sleep()
                NEW_TR = False

            PLAY = False
            userdata.rooms_out = userdata.rooms_in
            return 'goToNormal'



	

        
def main():
    
    global client
    try:
        rospy.init_node('commandManager')
        # move_base client
        client.wait_for_server()
        # Subscriber to the UIchatter topic
        UIsubscriber = rospy.Subscriber("UIchatter", String, UIcallback)
        # Subscriber to the newRoom topic 
        #newRoomSub = rospy.Subscriber("newRoom", String, newRoomDetected)

        # init the environment 
        rooms = Rooms()
        
        # Create a SMACH state machine
        sm = smach.StateMachine(outcomes=['init'])
        sm.userdata.connector = rooms

        # Open the container
        with sm:
            # Add states to the container
            smach.StateMachine.add('NORMAL', Normal(), 
                                transitions={'goToSleep':'SLEEP', 
                                                'goToPlay':'PLAY',
                                                'goToNormal':'NORMAL'},
                                    remapping={'rooms_in':'connector', 
                                            'rooms_out':'connector'})
            smach.StateMachine.add('SLEEP', Sleep(), 
                                transitions={'goToSleep':'SLEEP', 
                                                'goToNormal':'NORMAL'},
                                    remapping={'rooms_in':'connector', 
                                            'rooms_out':'connector'})
            smach.StateMachine.add('PLAY', Play(), 
                                transitions={'goToNormal':'NORMAL',
                                                'goToPlay':'PLAY'},
                                    remapping={'rooms_in':'connector', 
                                            'rooms_out':'connector'})


        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()

        outcome = sm.execute()

        rospy.spin()
        sis.stop()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")


if __name__ == '__main__':
    main()
