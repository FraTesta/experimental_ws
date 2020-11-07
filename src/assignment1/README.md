# Assignment 1 of Experimental Robotics Laboratory
## Indrodution
This is the first assignment of experimental robotics which consists of develop a basic software architecture that simulates a robotic dog.
This robot is able to receive position indicated via laser pointer and then reach such position, if and only if a person pronounces the word _play_ first. This mode of operation is called __PLAY__ mode . The robot can also work in two other different modes when it doesn't recive any  _play_ command: 
- __NORMAL__ mode where it moves in random positions.
- __SLEEP__ mode, the robot goes home (whihc is a known position).
## Software Architecture and System's Features
This software architecture uses three ROS nodes and just one service.
* __speakPerception__ is a ROS node which publishes a string "play" at random time. So it simulates a person saying the word _play_.
* __getPosition__ instead is a node which publishes a *geometry_msgs/2DPose* on a _/Position_ topic. It generates periodically random x and y positions, basically for two main purposes:
   1. Generate random position which simulates the pointing gesture of the user.
   2. Provide random position for the NORMAL mode. 
   It was chosen to mearge these two fitures since in this phase of the procject it's not yet required to implement a node dedicated to the reception and pre-processing of laser pointer data. So in order to avoid basically writing the same code twice, it was chosen to use only one node.
* __Navigation__ is the only service in the architecture and it has several purposes:
   1. Receives an x y position as a request which representing the goal position.
   2. Checks if the requested position is actually reachable (for istance if this position is inside the map).
   3. Simulates robot navigation just by letting time pass.
   4. Returns a response which contains the current x y position of the robot and a bool variable representing if the robot can really reach the position or not (true or false).
* __commandManager__ is the core node , that manages information from the two previous publishers and implements a _Finite State Machine_ which alternates the three possible states (__PLAY__,__NORMAL__ and __SLEEP__). Finally in according to them it makes some requests to the _Navigation_ service in order to move the robot. 
It should be noted that this node continuously receives positions from the _getPosition_ node but simply ignores such information when it is in a finite state where those data is not needed.

## Finite State Machine 
The finite state machine, present in the _commandManager_ node, was developed as follows: 
* __NORMAL__ : First of all _NORMAL_ is the initial state. In this state the _FSM_ checks if the _state_ variable (which contains the strings published from the _speakPerception_ node) is equal to the string _play_. In that case the _FSM_ goes in _PLAY_ state. Otherwise it makes a request to the _Navigation_ service to go in a random position provided by the _getPosition_ node. 
Then this routine is iterated for a random number of times (max 3 times) after which the robot goes in _SLEEP_ mode. 
* __PLAY__ : In this state the _commandManager_ moves the robot to the goal position which is just the last position sent by the _getPosition_ node. Then it call again the _Navigation_ service sending as goal position the user position, in order to tell the robot to come back to the user. After then that the _FSM_ come back in _NORMAL_ mode.
* __SLEEP__ : The robot goes to the home which is a known position a priori chosen. Then it sleeps for a random time after which the _FSM_ goes in NORML mode. 

## Installation
In order to use this package it's necessary to install the _smach_ library which allows to implement a _Finite State Machine_. So on a command window digit:
```

$ sudo apt-get install ros-kinetic-smach-viewer
```
Of coure it's also required to make a _catkin_make_ in your _ROS_ workspace
### File list
In this package you can find in the _src_ directory the three nodes file and the service. In the _launch_ directory has been defined several launch files to test the system (_assignment1.launch_ launches every components of the system). _srv_ directory contains the definition of the only service of the architecture. Finally you can find three _bash command file_ usefull to test the system avoiding to digit long terminals commands :
- __run_sys.sh__ allows to launch all componets of the system without the _getPosition_node
- __run_gesPosition.sh__ launches the _getPosition_ node
- __impossiblePosition.sh__  just publishes on the _Position_ topic a position messagge which is out of the map. This is usefull to test the response of the system in case the user points a position which is not reachable from the robot.     

## Usage
First of all remeber to source your wrokspace in every terminal you will use.
If wou want run the whole project just type:
```

$ roslaunch assignment1 assignment1.launch
```
### Test getPosition 
In order to test in two different terminals the messages random generates by the _getPosition_ node and see the their effects, plese go to the _assignment1_ directory and execute from shell:
```

$ cd ~/assignment1
$ ./run_getPosition.sh
```
and then in another terminal execute
```

$ cd ~/assignment1
$ ./run_sys.sh
```
You will see in first terminal window the positions randomly generated by _getPostion_ and the response in real time of the _commandManager_ node.
### Test impossible position
```

$ cd ~/assignment1
$ ./run_sys.sh
```
and in a new terminal execute
```

$ cd ~/assignment1
$ ./impossiblePosition.sh
```