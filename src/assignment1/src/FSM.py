#!/usr/bin/env python

from __future__ import print_function

import roslib
import rospy
import smach
import smach_ros
import time
import random
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from assignment1.srv import *

# INSTALLATION
# - create ROS package in your workspace:
#          $ catkin_create_pkg smach_tutorial std_msgs rospy
# - move this file to the 'smach_tutorial/scr' folder and give running permissions to it with
#          $ chmod +x state_machine.py
# - run the 'roscore' and then you can run the state machine with
#          $ rosrun smach_tutorial state_machine.py
# - install the visualiser using
#          $ sudo apt-get install ros-kinetic-smach-viewer
# - run the visualiser with
#          $ sudo apt-get install ros-kinetic-smach-viewer

X = 0
Y = 0
homeX = 10
homeY = 20

# 
def decision():
    return random.choice(['goToNormal','goToSleep'])

# callback for the get position subsriber
def callbackPos(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard x: %d  y: %d", data.linear.x, data.linear.y)
    global X
    X = data.linear.x
    global Y 
    Y = data.linear.y    

# client function
def navigation(x,y):

    rospy.wait_for_service('myNavigation')
    try:
        go_to = rospy.ServiceProxy('myNavigation',GoTo)
        check = go_to(x ,y)
        return check.o
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# define state NORMAL
class Normal(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep'],
                             input_keys=['unlocked_counter_in'],
                             output_keys=['unlocked_counter_out'])
        self.rate = rospy.Rate(1)  # Loop at 200 Hz
        self.counter = 0
        
    def execute(self,userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(3)
        global X
        global Y
        
        self.counter = random.randint(1,2)
        while not rospy.is_shutdown():  
            rospy.loginfo(rospy.get_caller_id() + 'Executing state NORMAL ')
            
            if self.counter == 4:
                return 'goToSleep'
            time.sleep(3)
            navigation(X,Y)
#            rospy.loginfo(rospy.get_caller_id() + 'i m going to x: %d y: %d',x, y)

            self.counter += 1
            

        return 'goToSleep' 
        
    

# define state SLEEP 
class Sleep(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep'],
                             input_keys=['locked_counter_in'],
                             output_keys=['locked_counter_out'])
        self.rate = rospy.Rate(200)  # Loop at 200 Hz

    def execute(self, userdata):

        time.sleep(random.randint(3,5))
        
        global homeX
        global homeY
        rospy.loginfo(rospy.get_caller_id() + 'Executing state SLEEP ')
        navigation(homeX,homeY)
#        rospy.loginfo(rospy.get_caller_id() + 'i m going to home x: %d y: %d',homeX,homeY)
        self.rate.sleep()
        return 'goToNormal'


        
def main():
    rospy.init_node('smach_example_state_machine')

    rospy.Subscriber("Position", Twist, callbackPos) # subsriber get_position 

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'goToSleep':'SLEEP', 
                                            'goToNormal':'NORMAL'},
                               remapping={'locked_counter_in':'sm_counter', 
                                          'locked_counter_out':'sm_counter'})
        smach.StateMachine.add('SLEEP', Sleep(), 
                               transitions={'goToSleep':'SLEEP', 
                                            'goToNormal':'NORMAL'},
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
