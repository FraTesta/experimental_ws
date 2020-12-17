# Assignment 2 

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)
## Indrodution
This project represents the next phase of the first assignment, that you can find in the __assignmet1__ directory. 
Basically the second assignment requires to implement a real robot model to be used in a gazebo simulation. The robot has to maintain the previous behaviors reguarding the _NORMAL_ and _SLEEP_ states of the FSM, but now the _PLAY_ state must be able to detect a green ball in the gazebo simulation and track it. When the robot reaches the ball, it must stop and turn its head left and right. After which it starts tracking the ball again as long as the ball is detected.
# Robot model 
  The robotic dog was developed in a very simple way, it has two classic wheels and a spherical one that allow it to move. I added a neck and a head on top of it, with a revolute joint that allows it to rotate 180 degrees.
  A camera has been applied to the robot's head in order to make the robot able to detect the ball.
  You can find the robot model description in the _urdf_ directory
# Software Architecture and System's Features
 Of course the software architecture has been updated mainly by replacing the previous ROS service dedicated to navigation, with an action server (_navigation_action_ sever ) capable of moving the robot within the Gazebo simulation. Finally the _getPosition_ and _speakPerception_ nodes have been replaced by a new ROS node dedicated to the detection and the tracking of the ball called _ballDetection_.
 You can launch the project 
 ```
 $ roslaunch assignment2 gazebo_world.launch
 ```
 and in another shell digit 
```
 $ rqt_graph 
```
 in order to see the structure of the new architecture.
 As you will see the project use two divverent namespaces to catalog the project nodes.
 _robotDog_ identifies all nodes concerning the robot control, while the _ball_ namespace identies the nodes of the ball. 
 
 Please notice that the __ball_state__ and __head_state__ topics are custom messages. The first one is used to communicate to the _commandManager_ when the ball has been detected or reached by the _ballDetection_ node. And the second one allows the _commandManager_ to notify to the _ballDetection_ when the head motions is finished.
 Those messages are necessary since the _PLAY_ routine provides the following steps:
1. When the _ballDetection_ node detects the ball, it communicates it to the _commandManager_ so that it can stop any server action in progress and switches to the PLAY state.
2. Once _ballDetection_ reaches the ball, it stops tracking the ball and notifies this to the _commandManager_, which will start the head motions.
3. When the robot's movements are finished, the _commandManager_ comunicate it to the _ballDetection_ node in order to start chasing the ball again, utill it is visible.

It was necessary to split the functions of the _PLAY_ state in these steps, since if we interrupt the detection loop of the ballDetection node (to perform the movements of the head) it would cause an interruption of the images provided by the camera. So it would cause less fluid images and sudden robot reactions, once the movements are finished.

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
           - 	_Gazebo_world.launch_ run the complete project instantiating the world and the robot, while the ball is hidden below the map. Therefore the robot will start in the NORMAL state until the ball is placed on its floor and detected. 
           - 	_Robot_model.launch_ launches just the robot model without any other control nodes.
- __msg__: contains the _ball_state.msg_ and _head_state.msg_ custom messages.
- __scripts__: holds the code of the following nodes:
            - _ballDetection.py_ implements the computer vision algorithm able to detect and track the ball.
            - _commandManager.py_ is the core the project since it define the FSM.
            - _navigation_action.py_ is a action server that allows the robot to move in a cartain position that is set as goal.
            - _go_to_point_ball.py_ is another action server dedicated to the movement of the ball
- __urdf__: floder that contains the definition of all models used by the Gazebo simulation
- __world__: contains the model of the world used in the Gazebo simulation
There are also some executable files to easily move the ball to certain positions, without having to publish those positions via shell commands when you test the project:
- _BallPostion1.sh_ moves the ball in position x = 1 and y = 1.
- _BallPostion2.sh_ moves the ball in position x = -5 and y = 3.
- _BallOff.sh_ put the ball under the ground of the world, in order to test when the ball disappears. 
