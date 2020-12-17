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
 Of course the software architecture has been updated  basically by replacing the previous ROS service dedicated to navigation, with an action server (_navigation_action_ sever ) capable of moving the robot within the Gazebo simulation. Finally the _getPosition_ and _speakPerception_ nodes have been replaced by a new ROS node dedicated to the detection and the tracking of the ball called _ballDetection_.
 You can launch the project 
 ```
 $ roslaunch assignment2 gazebo_world.launch
 ```
 and in another shell digit 
```
 $ rqt_graph 
```
 in order to understand the structure of the new architecture.
 As you will see the project use two divverent namespaces to catalog the project nodes.
 _robotDog_ identifies all nodes concerning the robot while the _ball_ namespace identies the nodes of the ball. 
 
 Please notice that the __ball_state__ and __head_state__ topics are custom messages. The first one is used to communicate to the _commandManager_ when the ball has been detected or reached by the _ballDetection_ node. And the second one allows the _commandManager_ to notify to the _ballDetection_ when the head motions is finished.
 Those messages are necessary since the _PLAY_ routine provides the following steps:
1. When the _ballDetection_ node detects the ball, it communicate it to the _commandManager_ so that it can stop any server action in progress and switch to the PlAY state.
2. Once _ballDetection_ reaches the ball it stops tracking the ball and it notifies this to the _commandManager_, which will start the head motions.
3. When the robot's movements are finished, the _commandManager_ comunicate it to the _ballDetection_ node in order to start chasing the ball again, utill it is visible.

It was necessary to split the functions of the Play state in these steps, since if we interrupt the detection loop of the ballDetection node (to perform the movements of the head) it would cause an interruption of the images provided by the camera. So this would cause less fluid images and sudden robot reactions, once the movements are finished.
