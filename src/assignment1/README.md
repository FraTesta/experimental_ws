# Assignment 1
## Indrodution
This is the first assignment of experimental robotics course which consists of developing a basic software architecture that simulates a robotic dog.
This robot is able to receive position indicated via laser pointer and then reach such position if and only if the user pronounces the word _play_ first. This mode of operation is called __PLAY__ mode . The robot can also work in two other different modes when it doesn't recive any  _play_ command, which are: 
- __NORMAL__ mode where the robot moves in random positions.
- __SLEEP__ mode, the robot goes home and sleeps for some time.
## Software Architecture and System's Features
This software architecture uses three ROS nodes and one service.
* __speakPerception__ is a ROS node which publishes a string "play" at random time. So it simulates a person saying the word _play_.
* __getPosition__ instead is a node which publishes a *geometry_msgs/2DPose* on the _/Position_ topic. It generates periodically random x and y positions, basically for two main purposes:
   1. Generate random position which simulates the pointing gesture of the user.
   2. Provide random positions for random navigation in the NORMAL mode. 
   It was chosen to mearge these two fitures since in this phase of the procject it's not yet required to implement a node dedicated to the reception and pre-processing of laser pointer data. So in order to avoid basically writing the same code twice, it was chosen to put them in the same node .
* __Navigation__ is the only service in the architecture and it has several purposes:
   1. Receives an x y position as request representing the goal position.
   2. Checks if the requested position is actually reachable (for istance if this position is inside the map).
   3. Simulates robot navigation just by letting time pass.
   4. Returns a response which contains the current x y position of the robot and a bool variable representing if the robot can really reach the position or not (true or false).
* __commandManager__ is the core node , that manages information from the two previous publishers and implements a _Finite State Machine_ which manages the three possible states (__PLAY__,__NORMAL__ and __SLEEP__). Finally in according to them it makes some requests to the _Navigation_ service in order to move the robot. 
It should be noted that this node continuously receives positions from the _getPosition_ node, but simply ignores such information when ther robot is in a finite state where those data are not needed.

However you can take a look to the software architecture typing:
```
$ rqt_graph
```
when the system is running. 

## Finite State Machine 
The finite state machine which is implemented the _commandManager_ node, has been developed with the following states: 
* __NORMAL__ : is the initial state. This state checks if the _state_ variable (which contains the strings published from the _speakPerception_ node) is equal to the string _play_, in that case the _FSM_ goes in _PLAY_ state. Otherwise it makes a request to the _Navigation_ service to go in a random position provided by the _getPosition_ node. 
Then this routine is iterated for a random number of times (max 4 times) after which the robot goes in _SLEEP_ mode. 
* __PLAY__ : In this state the _commandManager_ moves the robot to the goal position which is just the last position sent by the _getPosition_ node. Then it call again the _Navigation_ service sending the user position as goal position, in order to tell the robot to come back to the user. After then that the _FSM_ come back in _NORMAL_ mode.
* __SLEEP__ : The robot goes to the home which is a known position a priori chosen. Then it sleeps for a random time after which the _FSM_ goes in NORML mode. 

You can see and check the behaviors of the _FSM_ typing in a terminal window :
```
$ rosrun smach_viewer smach_viewer.py
```

## Installation
In order to use this package it's necessary to install the _smach_ library which allows to implement a _Finite State Machine_. So on a command window digit:
```
$ sudo apt-get install ros-kinetic-smach-viewer
```
Of coure it's also required to make a _catkin_make_ in your _ROS_ workspace
### File list
In this package you can find in the _src_ directory containg the three nodes file and the service. In the _launch_ directory has been defined several launch files to test the system (_assignment1.launch_ launches every components of the system). _srv_ directory contains the definition of the only service of the architecture. Finally you can find three _bash command file_ usefull to test the system avoiding to digit long terminals commands :
- __run_sys.sh__ allows to launch all componets of the system without the _getPosition_node
- __run_gesPosition.sh__ launches only the _getPosition_ node
- __run_impossiblePosition.sh__  just publishes on the _Position_ topic a position messagge which is out of the map. This is usefull to test the response of the system in case the user points a position which is not reachable from the robot.    


## Usage
First of all __remeber to source your workspace in every terminal you will use.__

If wou want to run the whole project just type:
```
$ roslaunch assignment1 assignment1.launch
```
Remeber that you can check the current state of the _FSM_ typing :
```
$ rosrun smach_viewer smach_viewer.py
```
However in the terminal you can see the following logs:
- __It's going into position x = .. and y = ..__ which is actually a log of the _Navigation_ node 
- __The robot is arrived at..__ 
- __I heard _play___ which means that the _commandManager_ has recived a string _play_ from the _speakPerception_ node

All the software components have been developed trying to simulate a possible real usege of this system, using many random variables for: process execution times, positions generated and commands given by the user. However, executables dedicated to testing some features of the project have been added.
### 1. Test getPosition 
In order to test in two different terminals the messages random generates by the _getPosition_ node and see their effects, please execute this commands from your shall:
```
$ roscd assignment1/
$ ./run_getPosition.sh
```
and then in another terminal execute
```
$ roscd assignment1/
$ ./run_sys.sh
```
You will see in first terminal the positions randomly generated by _getPostion_ and in the second one the response in real time of the _commandManager_ node. Please notice that most of the generated positions are ignored by the _commandManager_, since either the robot is not in a _state_ which can use them, or it is already going to a certain position (so when there is the _i'm going to ..._ log)
### 2. Test impossible position
```
$ roscd assignment1/
$ ./run_sys.sh
```
and in a new terminal execute
```
$ roscd assignment1/
$ ./impossiblePosition.sh
```
In this way you will send a goal position in x = 23 and y = 5 to the _commandManager_ node, and since that the map has been a priori set with max dimension x = 20 and y = 20. Then you will see the log : _The robot cannot reach that position_ . Remeber that _getPosition_ simulates both user pointing and random navigation of the robot, so you are testing both. 

## System's limitation and possible improvements
### Limitation
As already mentioned there are some position which currently must be configured in advance such as:
- _Home_ position: is currently set in position x = 10 and y = 20 as you can see in the _commandManager_ (__homeX__ and __homeY__ global variables)
- _User position_: since the robot is currently not able to localize person , therefore it was necessary to give the system a known position for it and of course we assume that the person always remains in the same position.
- _Map_:  was defined simply with two parameters indicating the maximum dimension along the x and y axes (__Xmax__ and __Ymax__ in the _Navigation_ code) which is sufficent since it is just a 2D map . No obstacles have been added since it was not required to implement an obstacle avoidance algorithm. 

Regarding the _FSM_ was necessary to add some constraints which consist of : 
When ther robot is in a SLEEP mode it cannot do anything utill the sleeptime is over. So even if the _commandManager_ recives a _play_ command from the _speackPerception_ publisher it could not go to _PLAY_ mode. Another limitation is that currently is not possible to remain in the _PLAY_ state without first returning to the NORMAL state (for istance sending a _play_ string from _speakPerception_).

### Possible improvements:
- Improve the _getPosition_ node with a drive which is really able to percive pointing gesture positions. And maybe move the currently random postion generator in the _commandManager_ node for maintaining the random navigation fiture. 
- Give the possibility to the user to change the home and his/here positions. 
- Implement a proper simulation environment. 
- Add a more efficent and realistic navigation algorithm and/or a SLAM algo.
- Develop drivers to actually receive sensor data also from a microphone.
## Contacts
 Francesco Testa francesco.testa.ge@gmail.com
