# Assignment 1 Experimental Robotic
## Indrodution
This is the first assignment of experimental robotics which consists of develop a basic software architecture that simulates a robotic dog.
This robot is able to receive position indicated via laser pointer and then reach such position if and only if a person pronounces the word "play" first. This mode of operation is called PLAY mode . The robot can also work in two other different modes when it doesn't recive a command play: 
- NORMAL mode where it moves in random position
- SLEEP mode, it go to home (known position)
## Architecture 
This software architecture uses three ROS node and just one service.
* _speakPerception_ is a ROS node which publishes a string "play" at random time. So it simulates a person saying the word play.
* *getPosition* instead is a node which publishes a _geometry_msgs/2DPose_