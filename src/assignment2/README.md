# Assignment 2 

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)
## Indrodution
This project represents the next phase of the first assignment, that you can find in the __assignmet1__ directory. 
Basically the second assignment requires to implement a real robot model to be used in a gazebo simulation. The robot has to maintain the previous behaviors reguarding the _NORMAL_ and _SLEEP_ states of the FSM, but now the _PLAY_ state must be able to detect a green ball in the gazebo simulation and track it. When the robot reaches the ball, it must stop and turn its head left and right. After which it starts tracking the ball again as long as the ball is detected.

## Robot model 
  The robotic dog was developed in a very basic way, it has two classic wheels and a spherical one that allow it to move. I added a neck fixed to chassis and a head has been connected on top of the neck with a revolute joint, that allows it to rotate 180 degrees. I used the transmission package in order to control the head joint. 
  A camera has been applied to the robot's head in order to make the robot able to detect the ball. For the camera has been used a plug-in that you can easly find on this tutorial http://gazebosim.org/tutorials?tut=ros_gzplugins. Then ears were added just for fun.
  In the xacro file you will find some macros but thay should be quite intuitive. 
  The _robotDog.gazebo_ contains the definitions of the camera plugin and the differential driver plugin which allows to control the robot wheels. 
In the _urdf_ directory there is also the model description of the ball and the human (which has no active function).  
## Software Architecture and System's Features
 Of course the software architecture has been updated, mainly by replacing the previous ROS service dedicated to navigation, with an action server (_navigation_action_ sever ) capable of moving the robot within the Gazebo simulation. Finally the _getPosition_ and _speakPerception_ nodes have been replaced by a new ROS node dedicated to the detection and the tracking of the ball, called _ballDetection_.
 You can launch the project 
 ```
 $ roslaunch assignment2 gazebo_world.launch
 ```
 and in another shell digit 
```
 $ rqt_graph 
```
 in order to see the structure of the new architecture.
 As you will see the project uses two different namespaces to catalog the project nodes.
 _robotDog_ identifies all nodes concerning the robot control, while the _ball_ namespace identies the nodes of the ball. 
 
 Please notice that the __ball_state__ and __head_state__ topics are custom messages. The first one is used to communicate to the _commandManager_ whenever the ball has been detected or reached by the _ballDetection_ node. And the second one allows the _commandManager_ to notify to the _ballDetection_ when the head motions is finished.
 Those messages are necessary since the _PLAY_ routine provides the following steps:
1. When the _ballDetection_ node detects the ball, communicates it to the _commandManager_ so that it can stop any server actions in progress and switches to the PLAY state.
2. Once _ballDetection_ reaches the ball, it stops tracking the ball and notifies this to the _commandManager_, which will start the head motions.
3. When the robot's movements are finished, the _commandManager_ send a _head_state_ message to the _ballDetection_ node, in order to start chasing the ball again, utill the ball is visible.
#### Architectural choices
It was chosen to split the features of the _PLAY_ state as explained in the previous points, in order to keep the ball detection continuous and without interruptions. In particular if we stop the loop of ball detection in the _ballDetection_ node, to turn the head of the robot, then the images freeze. After the head movements the images suddenly update, causing abrupt position corrections or the stalling of the robot. Morover this choice improve the synchronization between the two nodes. Another possible solution was to dedicate just one node only for ball detection and another node for ball tracking. This may be reasonable but it increase the number of nodes and so worsening system performance. 

#### Ball tracking 
  As we said the _ballDetection_ node implements some computer vision features, provided by the openCV library. In particular allows the node to find the center of a green ball and then draw a cricle around it. 
  Those informations are used to compute the velocities that should apply to the robot motors in order to reach the ball. So the radius of the circle is used to compute the distance from the ball, for instance if the radius is small, this means that the ball is still far away. While the center is used to correct the yaw of the robot which will try to keep the center of ball in the center of the camera images. 
  If the radius of the ball is greater than a certain value and the center of the ball minus the center of the width of the image is close to 0, than we can consider the ball as reached. This is the condition behind the implementation of the PLAY state. 

 ## Installation 
 In order to use this package please install:
 The __smach__ library to use the FSM.
  ```
  $ sudo apt-get install ros-kinetic-smach-viewer
  ```
 The __ros_control__ package to control the joint of the neck.
```
$ sudo apt-get install ros-[ROS_version]-ros-control ros-[ROS_version]-ros-controllers
$ sudo apt-get install ros-[ROS_version]-gazebo-ros-pkgs ros-[ROS_version]-gazebo-ros-control
```
The __openCV__ library to use the computer vision features
```
$ sudo apt-get install python-opencv
```
The __numpy__ library to use some image processing features provided by the camera
```
$ pip install numpy
```
It is also necessary to have installed _ROS kinetic_ and _Gazebo 7_ correctly.

## File list
 In the package you will find the following foulder:
 - __action__: which contains the definition of file.action files (Planning.action and Planning_ball.action) of the two action server : _navigation_action_ and go_to_point_ball.
 - __config__: contains the _motor_robot_dog.yaml_ file which defines the joint controller parameters
 - __launch__: contains some launch files:
           1. _Gazebo_world.launch_ run the complete project instantiating the world and the robot, while the ball is hidden below the map. Therefore the robot will start in the NORMAL state until the ball is placed on its floor and detected. 
           2. _Robot_model.launch_ launches just the robot model without any other control nodes.
           3. _test.launch_ runs also a node dedicated to move the ball in random positions, so it's very nice to test the system
- __msg__: contains the _ball_state.msg_ and _head_state.msg_ custom messages.
- __scripts__: holds the code of the following nodes:
            1. _ballDetection.py_ implements the computer vision algorithm able to detect and track the ball.
            2. _commandManager.py_ is the core the project since it define the FSM.
            3. _navigation_action.py_ is a action server that allows the robot to move in a cartain position that is set as goal.
            4. _go_to_point_ball.py_ is another action server dedicated to the movement of the ball.
            5. _randBallPos.py_ action client to move the ball in random positions.
- __urdf__: floder that contains the definition of all models used by the Gazebo simulation
- __world__: contains the model of the world used in the Gazebo simulation
#### Exectuables for testing
There are also some executable files to easily move the ball to certain positions, without having to publish those positions via shell commands when you test the project:
- _ballPosition1.sh_ moves the ball in position x = 1 and y = 1.
- _ballPosition2.sh_ moves the ball in position x = -5 and y = 3.
- _ballPosition3.sh_ moves the ball in posotion x= -4 and y = -1
- _BallOff.sh_ put the ball under the ground of the world, in order to test when the ball disappears. 
Finally you can launch _test.launch_ file which moves the ball in random position in order to stress the system. It also make dessapear the ball after some time. You will see that the robot overturns if the ball hits it but I think it is quite normal behavior.
 
## System's limitation and possible improvements
There are some problems related to the velocities sent by the _ballDetection_ node to the robot. In particular, if the ball suddenly moves towards the robot, then high speed commands might be applied to the motors causing the robot to overturn.
This should be easily solved by implementing a simple PID controller with the appropriate. parameters to be applied to the differential driver motors of the robot.
Moreover sometimes the robot takes a some time to understand that the goal is actuallt reached, but it is quite rare that this happens and in any case after some corrections the robot realizes it.
Finally another problem is that the robot always keeps its head straight when moving in the NORMAL and in SLEEP state. This makes his ability to detect the ball rather limited.So a possible future improvements may be Implent a feature that allows the robot to look around while it is in NORMAL and SLEEP mode, in order to improve its ability of detecting rhe ball. So a possible future improvements may be Implent a feature that allows the robot to look around while it is in NORMAL and SLEEP mode, in order to improve its ability of detecting rhe ball.
## Contacts
Francesco Testa francesco.testa.ge@gmail.com
