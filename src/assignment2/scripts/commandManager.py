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
from assignment2.msg import HeadState



## X position of the home 
# Here you can set the a priori X position of the house
homeX = -5
## Y position of the home 
# Here you can set the a priori Y position of the house
homeY = 7
## State variables
## Variable to check whether the ballDetection node has detected a ball 
ballDetected = False
## Control variable to avoid false readings
ballCheck = False
## Variable that communicates whether the robot has reached the ball
ballReached = False


##Action client for Navigation action server
client = actionlib.SimpleActionClient('robot_reaching_goal', motion_plan.msg.PlanningAction)
# AATTT ho messo wait server nel main


## This function chose randomly the next state of the FSM
def decision():
    return random.choice(['goToNormal','goToSleep'])
 

## 
def callbackBall(data):
    '''Callback function for the ballDetection subsriber.
    Which recives and handle a ball_state msg. It update the local variable and each time it receives a msg that a ball is detected after some time, interrupts any
    action server goal. So that the robot stops and switchs into the PLAY state '''
    global ballDetected, ballCheck, ballReached
    ballDetected = data.ballDetected
    ballReached = data.ballReached
#    rospy.loginfo( ballCheck)
    if ballDetected == True and ballCheck == False:
	ballCheck = True
        rospy.loginfo("Ball detected !!, current action interrupt")
	client.cancel_all_goals() 

class Normal(smach.State):
    '''This class defines the NORMAL state of the FSM. In particular It sends random position to the navigation_action server
    and it checks whether a ball is detected in order to move to PLAY state.
    Otherwise after some iterations it goes in SLEEP mode '''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep','goToPlay'])
        self.rate = rospy.Rate(1)  # Loop at 200 Hz
	## Counter variable to check the number of iteration of the NORMAL state in order to move to SLEEP state after a certain number 
        self.counter = 0
        
    def execute(self,userdata):

        global ballDetected
        
        self.counter = random.randint(1,2) 
        # Creates a goal to send to the action server.
        goal = motion_plan.msg.PlanningGoal()

        while not rospy.is_shutdown():  

            if ballCheck == True and ballDetected == True:
		rospy.loginfo("Start to track the ball")
                return 'goToPlay'
            if self.counter == 4:
                return 'goToSleep'           
            # request for the service to move in X and Y position
	    xRandomGoal = random.randrange(1,5,1)
	    yRandomGoal = random.randrange(1,5,1)
	    rospy.loginfo("I'm going to position x = %d y = %d", xRandomGoal, yRandomGoal)
            goal.target_pose.pose.position.x = xRandomGoal
            goal.target_pose.pose.position.y = yRandomGoal
	    client.send_goal(goal)
            client.wait_for_result()
	    rospy.loginfo("Goal reached")
            time.sleep(2)
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
        global homeX
        global homeY

        rospy.loginfo("I m in SLEEP mode")
        goal = motion_plan.msg.PlanningGoal()
        goal.target_pose.pose.position.x = homeX
        goal.target_pose.pose.position.y = homeY
        client.send_goal(goal)

        client.wait_for_result()       
	rospy.loginfo("Home reached")
        time.sleep(random.randint(3,6))
#        self.rate.sleep()
        return 'goToNormal'


class Play(smach.State):
    '''Class that defines the PLAY state. 
     It move the robot in X Y location and then asks to go back to the user.'''
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToPlay'])
        
        self.rate = rospy.Rate(200)
	self.joint_pub = rospy.Publisher("joint_head_controller/command",Float64,queue_size=1)  
	self.headState = rospy.Publisher("head_state",HeadState, queue_size = 1)

    def execute(self, userdata):

        rospy.loginfo("I m in PLAY mode")
	global ballDetected, ballCheck, ballReached

	while True:
             if(ballDetected == False): 
		ballCheck = False
		rospy.loginfo("Ball lost")
                return 'goToNormal' 
	     if(ballReached == True):
                rospy.loginfo("ball reached !!!")
		self.joint_pub.publish(0.785398) 
		time.sleep(5)
		self.joint_pub.publish(-0.785398)
		time.sleep(5)
		self.joint_pub.publish(0)
		time.sleep(5)
		rospy.loginfo("Finito di muovere la testa ")
		headMsg = HeadState()
		headMsg.HeadMotionStop = True
		self.headState.publish(headMsg) 
                
	     time.sleep(3)      
	

        
def main():
    rospy.init_node('smach_state_machine')
     
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
