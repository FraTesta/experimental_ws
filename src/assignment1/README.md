# Assignment 1 Experimental Robotic
## Indrodution
This is the first assignment of experimental robotics which consists of develop a basic software architecture that simulates a robotic dog.
This robot is able to receive position indicated via laser pointer and then reach such position if and only if a person pronounces the word "play" first. This mode of operation is called __PLAY__ mode . The robot can also work in two other different modes when it doesn't recive a command play: 
- __NORMAL__ mode where it moves in random position
- __SLEEP__ mode, the robot go to home (known position)
## Architecture 
This software architecture uses three ROS node and just one service.
* __speakPerception__ is a ROS node which publishes a string "play" at random time. So it simulates a person saying the word play.
* __getPosition__ instead is a node which publishes a *geometry_msgs/2DPose* on a _/Position_ topic. It generates periodically random x and y positions, basically for two main purposes:
   1. Generate random position which simulates the pointing gesture of the user.
   2. Provide random position for the NORMAL mode. 
* __Navigation__ is the only one service in the architecture, it has several purposes:
   1. Receives an x y position as a request which representing the goal position.
   2. Checks if the requested position is actually reachable (for istance if this position is inside the map).
   3. Simulates robot navigation just by letting time pass.
   4. Returns a response which contains the current x and y position of the robot and a bool variable representing if the robot can reach the position or not (true or false).
* __commandManager__ is the core node , that manages information from the two previous publishers and implements a _Finite State Machine_ which alternates and manages the three possible states and in according to them it makes some requests to the _Navigation_ service.
