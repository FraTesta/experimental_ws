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

x = 3
y = 5
homeX = 10
homeY = 20
state = "NoInfo"

def decision():
    return random.choice(['goToNormal','goToSleep'])

def callbackSta(data): 
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global state 
    state = "play"


# define state Unlocked
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
        # function called when exiting from the node, it can be blacking
        global x
        global y
        global state
        
        self.counter = 0
        while not rospy.is_shutdown():  
            rospy.loginfo(rospy.get_caller_id() + 'Executing state NORMAL ')
#       userdata.unlocked_counter_out = userdata.unlocked_counter_in + 1
            time.sleep(2)
            if self.counter == 3:
                return 'goToSleep'
            self.rate.sleep()
#            navigation(x,y)
            rospy.loginfo(rospy.get_caller_id() + 'i m going to x: %d y: %d',x, y)
            x = x + 1
            y = y + 1
            if state == "play":
                state = 'noInput'
                return 'goToPlay'

            self.counter += 1
            
        
        return decision()
    

# define state Locked
class Sleep(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['goToNormal','goToSleep'],
                             input_keys=['locked_counter_in'],
                             output_keys=['locked_counter_out'])
        self.sensor_input = 0
        self.rate = rospy.Rate(200)  # Loop at 200 Hz

    def execute(self, userdata):
        # simulate that we have to get 5 data samples to compute the outcome
        time.sleep(5)
        #            navigation(x,y)
        global homeX
        global homeY
        rospy.loginfo(rospy.get_caller_id() + 'Executing state SLEEP ')
        rospy.loginfo(rospy.get_caller_id() + 'i m going to home x: %d y: %d',homeX,homeY)
        self.rate.sleep()
        return 'goToNormal'

# define state Play
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
        time.sleep(3)
        #            navigation(x,y)

        rospy.loginfo(rospy.get_caller_id() + 'Executing state PLAY ')
        

        return 'goToNormal'


        
def main():
    rospy.init_node('smach_example_state_machine')

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
