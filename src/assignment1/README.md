# Assignment 1 Experimental Robotic
## Indrodution
This is the first assignment of experimental robotics which consists of develop a basic software architecture that simulates a robotic dog.
This robot is able to receive position indicated via laser pointer and then reach such position, if and only if a person pronounces the word _play_ first. This mode of operation is called __PLAY__ mode . The robot can also work in two other different modes when it doesn't recive any  _play_ command: 
- __NORMAL__ mode where it moves in random position
- __SLEEP__ mode, the robot go to home (known position)
## Software Architecture 
This software architecture uses three ROS node and just one service.
* __speakPerception__ is a ROS node which publishes a string "play" at random time. So it simulates a person saying the word play.
* __getPosition__ instead is a node which publishes a *geometry_msgs/2DPose* on a _/Position_ topic. It generates periodically random x and y positions, basically for two main purposes:
   1. Generate random position which simulates the pointing gesture of the user.
   2. Provide random position for the NORMAL mode. 
* __Navigation__ is the only service in the architecture and it has several purposes:
   1. Receives an x y position as a request which representing the goal position.
   2. Checks if the requested position is actually reachable (for istance if this position is inside the map).
   3. Simulates robot navigation just by letting time pass.
   4. Returns a response which contains the current x y position of the robot and a bool variable representing if the robot can reach the position or not (true or false).
* __commandManager__ is the core node , that manages information from the two previous publishers and implements a _Finite State Machine_ which alternates the three possible states (__PLAY__,__NORMAL__ and __SLEEP__). Finally in according to them it makes some requests to the _Navigation_ service in order to move the robot.

## Finite State Machine 
The finite state machine, present in the _commandManager_ node, was developed as follows: 
* __NORMAL__ : First of all _NORMAL_ is the initial state. In this state thr _FSM_ checks if the _state_ variable (which contains the strings published from the _speakPerception_ node) is equal to the string _play_. In that case changes the _FSM_ in _PLAY_ state. Otherwise it makes a request to the _Navigation_ service to go in a random position provided by the _getPosition_ node. 
Then this routine is iterated for a random number of times (max 3 times) after which the robot goes in _SLEEP_ mode. 
* __PLAY__ : In this state the _commandManager_ moves the robot to the goal position which is just the last /Position sent by the _getPosition_ node. Then it call again the _Navigation_ service sending as goal position the user position, in order to tell the robot to come back to the user. After then that the _FSM_ come back in _NORMAL_ mode.
* __SLEEP__ : The robot goes to the home which is a known position a priori chosen. Then it sleeps for a random time after which the _FSM_ goes in NORML mode. 