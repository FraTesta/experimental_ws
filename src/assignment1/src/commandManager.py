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
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D

from assignment1.srv import *

## robot X position variable
# @param X is the initialize X position of the robot
X = 0  
## robot Y position variable
# @param Y is the initialize Y position of the robot 
Y = 0
## X position of the home 
# @param homeX here you can set the a priori X position of the house
homeX = 10
## Y position of the home 
# @param homeY here you can set the a priori Y position of the house
homeY = 20
## State variable
# @param state This is the state coming from State node (that's why is a string) and it can be ether play or NoInfo 
state = "NoInfo"

## User Position X
# @param personX it represents the x position of the user
personX = 2
## User Position Y
# @param personX it represents the x position of the user
personY = 3

## This function chose randomly the next state of the FSM
def decision():
    return random.choice(['goToNormal','goToSleep'])

## This callback function for the get position subsriber. 
# It recives a geometry_msgs/2DPose message which contains a X and Y position from the getPosition node
def callbackPos(data):
#    rospy.loginfo(rospy.get_caller_id() + " I heard x: %d  y: %d", data.x, data.y)
    global X
    X = data.x
    global Y 
    Y = data.y    

## Callback function for the speckPerception subsriber.
# Which recives a string from   
def callbackSta(data): 
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
    global state 
    state = "play"

## Client function for the Navigation service which makes a request to go in X and Y position and menages the Navigation service responses.
#  It prints some log messages which communicate to the user the position in which the robot has arrived or if cannot reach that position. 
def navigation(x,y):

    global homeX
    global homeY
    global personX
    global personY
    rospy.wait_for_service('myNavigation')
    try:
        go_to = rospy.ServiceProxy('myNavigation',GoTo)
        check = go_to(x ,y)
        if check.ok == True:
            if (check.currentX ==homeX) & (check.currentY ==homeY):
                rospy.loginfo(rospy.get_caller_id() + " The robot is arrived at home" )
            elif (check.currentX == personX) & (check.currentY == personY):
                rospy.loginfo(rospy.get_caller_id() + " The robot is returned to the user" )
            else:
                rospy.loginfo(rospy.get_caller_id() + " The robot is arrived in position x: %d , y: %d", check.currentX, check.currentY )
             
        else: 
            rospy.loginfo(rospy.get_caller_id() + " The robot cannot reach that posiion")
        return check.ok
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

## This class defines the NORMAL state of the FSM. 
# At each execution it randomly chooses how many movements the robot will make and put this number in the counter variable.
# Then there is a while loop which checks:
# - If there is a "play" message, then the FSM must go in PLAY state 
# - If it has been iterated as many times as the counter variable then it goes in SLEEP state
# Otherwise it make a rquest to the Navigation node to go in X and Y position
class Normal(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep','goToPlay'])
        self.rate = rospy.Rate(1)  # Loop at 200 Hz
        self.counter = 0
        
    def execute(self,userdata):

        global X
        global Y
        global state
        
        self.counter = random.randint(1,2) 
        while not rospy.is_shutdown():  
            time.sleep(1)
            if state == "play":
                state = 'noInput'
                return 'goToPlay'
            if self.counter == 4:
                return 'goToSleep'           
            navigation(X,Y) # request for the service to move in X and Y position

            self.counter += 1
            
        return 'goToSleep' 
        
    

## It defines the SLEEP state which sleeps for a random period of time.
# Then it makes a request to the Navigation service to go to the home location.
# Finally it returns in the NORMAL state
class Sleep(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep'])
        self.rate = rospy.Rate(200)  # Loop at 200 Hz

    def execute(self, userdata):

        time.sleep(random.randint(1,4))
        
        global homeX
        global homeY
        navigation(homeX,homeY) 
        self.rate.sleep()
        return 'goToNormal'

## Class that defines the PLAY state. 
# It move the robot in X Y location and then asks to go back to the user.
class Play(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToPlay'])
        
        self.rate = rospy.Rate(200)  

    def execute(self, userdata):
        global X
        global Y 

        navigation(X,Y)
        navigation(personX,personY)
        time.sleep(3)

        return 'goToNormal'       


        
def main():
    rospy.init_node('smach_example_state_machine')

    rospy.Subscriber("Position", Pose2D, callbackPos) # subsriber get_position 
    rospy.Subscriber("StateString", String, callbackSta)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'goToSleep':'SLEEP', 
                                            'goToPlay':'PLAY',
                                            'goToNormal':'NORMAL'})
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'goToSleep':'SLEEP', 
                                            'goToNormal':'NORMAL'})
        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'goToNormal':'NORMAL',
                                            'goToPlay':'PLAY'})


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()