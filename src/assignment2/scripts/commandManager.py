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
import motion_plan.msg
import actionlib
import actionlib_tutorials.msg
import random 
from std_msgs.msg import String
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from assignment2.msg import Ball_state


from assignment2.srv import *

## robot X position variable
# @param X is the initialize X position of the robot
X = 0  
## robot Y position variable
# @param Y is the initialize Y position of the robot 
Y = 0
## X position of the home 
# @param homeX here you can set the a priori X position of the house
homeX = -5
## Y position of the home 
# @param homeY here you can set the a priori Y position of the house
homeY = 7
## State variable
# @param state This is the state coming from State node (that's why is a string) and it can be ether play or NoInfo 
ballDetected = False
ballCheck = False



## User Position X
# @param personX it represents the x position of the user
personX = 0.2
## User Position Y
# @param personX it represents the x position of the user
personY = 0

##init action client for Navigation action server
client = actionlib.SimpleActionClient('/robot_reaching_goal', motion_plan.msg.PlanningAction)
# AATTT ho messo wait server nel main

## This function chose randomly the next state of the FSM
def decision():
    return random.choice(['goToNormal','goToSleep'])
 

## Callback function for the ballDetection subsriber.
# Which recives and handle a ball_state msg   
def callbackBall(data):
    global ballDetected
    currentBallDetected = data.ballDetected
    if ballDetected == True and ballCheck == False:
	ballCheck = True
        rospy.loginfo("Ball detected , current action interrupt")
	client.cancel_all_goals() 

class Normal(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep','goToPlay'])
        self.rate = rospy.Rate(1)  # Loop at 200 Hz
	## @param joint_pub to move the head of the robot 
        self.joint_pub = rospy.Publisher("joint_head_controller/command",Float64,queue_size=1)
        self.counter = 0
        
    def execute(self,userdata):

        global ballDetected
        
        self.counter = random.randint(1,2) 
        # Creates a goal to send to the action server.
        goal = motion_plan.msg.PlanningGoal()

        while not rospy.is_shutdown():  

            if ballDetected == True:
		rospy.loginfo("Ball detected")
		#client.cancel_goal()
                return 'goToPlay'
            if self.counter == 4:
                return 'goToSleep'           
            # request for the service to move in X and Y position
            goal.target_pose.pose.position.x = random.randrange(1,5,1)
            goal.target_pose.pose.position.y = random.randrange(1,5,1)
	    client.send_goal(goal)
            client.wait_for_result()
#            result = client.result()
#            rospy.loginfo("I m arrived at x = %f y = %f", result)
#           add possible print message ,for goal reached 
            time.sleep(4)
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
        global homeX
        global homeY

        rospy.loginfo("I m in SLEEP mode")
        goal = motion_plan.msg.PlanningGoal()
        goal.target_pose.pose.position.x = homeX
        goal.target_pose.pose.position.y = homeY
        client.send_goal(goal)

        client.wait_for_result()       
        time.sleep(random.randint(3,6))
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

        rospy.loginfo("I m in PLAY mode")

	while True:
             if(ballDetected == False): 
		ballCheck = False
                return 'goToNormal' 
	     time.sleep(3)      
	

        
def main():
    rospy.init_node('smach_example_state_machine')
     
    rospy.Subscriber("ball_state",Ball_state, callbackBall)
    client.wait_for_server()

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
