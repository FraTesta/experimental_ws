#!/usr/bin/env python2

## @package CommandManager
#  This node include the subsription to State and GetPosition publishers,
#  And implement a finite state machine 
#  which manages the information coming from the two nodes and changes the state of the system in according to it.
 
## Documentation for a function.
#
#  More details.

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

personX = 2
personY = 3

def decision():
    return random.choice(['goToNormal','goToSleep'])

## callback for the get position subsriber
def callbackPos(data):
#    rospy.loginfo(rospy.get_caller_id() + " I heard x: %d  y: %d", data.x, data.y)
    global X
    X = data.x
    global Y 
    Y = data.y    

## callback for the speckPerception subsriber 
def callbackSta(data): 
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
    global state 
    state = "play"

## client function for navigation service 
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

## defines the NORMAL state
class Normal(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep','goToPlay'],
                             input_keys=['unlocked_counter_in'],
                             output_keys=['unlocked_counter_out'])
        self.rate = rospy.Rate(1)  # Loop at 200 Hz
        self.counter = 0
        
    def execute(self,userdata):

        global X
        global Y
        global state
        
        self.counter = random.randint(1,2) # it determins how many movements should do the robot in NORMAL mode
        while not rospy.is_shutdown():  
 #           rospy.loginfo(rospy.get_caller_id() + 'Executing state NORMAL ')
            time.sleep(1)
            if state == "play":
                state = 'noInput'
                return 'goToPlay'
            if self.counter == 4:
                return 'goToSleep'           
            navigation(X,Y) # request for the service to move in X and Y position

            self.counter += 1
            
        return 'goToSleep' 
        
    

## defines the SLEEP state 
class Sleep(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep'],
                             input_keys=['locked_counter_in'],
                             output_keys=['locked_counter_out'])
        self.rate = rospy.Rate(200)  # Loop at 200 Hz

    def execute(self, userdata):

        time.sleep(random.randint(1,4))
        
        global homeX
        global homeY
 #       rospy.loginfo(rospy.get_caller_id() + ' Executing state SLEEP ')
        navigation(homeX,homeY) # move in home position 
        self.rate.sleep()
        return 'goToNormal'

## defines the PLAY state
class Play(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToPlay'],
                             input_keys=['locked_counter_in'],
                             output_keys=['locked_counter_out'])
        
        self.sensor_input = 0
        self.rate = rospy.Rate(200)  # Loop at 200 Hz

    def execute(self, userdata):
        # simulate that we have to get 5 data samples to compute the outcome
        global X
        global Y 

        navigation(X,Y)
        navigation(personX,personY)


 #       rospy.loginfo(rospy.get_caller_id() + ' Executing state PLAY ')
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
                                            'goToNormal':'NORMAL'},
                               remapping={'locked_counter_in':'sm_counter', 
                                          'locked_counter_out':'sm_counter'})
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'goToSleep':'SLEEP', 
                                            'goToNormal':'NORMAL'},
                               remapping={'unlocked_counter_in':'sm_counter',
                                          'unlocked_counter_out':'sm_counter'})
        smach.StateMachine.add('PLAY', Play(), 
                               transitions={'goToNormal':'NORMAL',
                                            'goToPlay':'PLAY'},
                               remapping={'unlocked_counter_in':'sm_counter',
                                          'unlocked_counter_out':'sm_counter'})


    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()