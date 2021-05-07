![Unige Logo](https://raw.githubusercontent.com/FraTesta/experimental_ws/master/src/final_assignment/documentation/doc_pages/unige_stemma.png)

# __Final Assignment__
## **Table Of Contents**
  - [__Introduction__](#introduction)
  - [__Software Architecture__](#software-architecture)
    - [Description](#description)
    - [**Architecture Choices**](#architecture-choices)
    - [**ROS msgs**](#ros-msgs)
  - [**System's Features**](#systems-features) 
    - [**Move Base and Gmapping settings**](#move-base-and-gmapping-settings)
    - [**Robot Model & Knowledge Rappresentation**](#robot-model--knowledge-rappresentation)
    - [__FSM Description__](#fsm-description)
    - [**Explore**](#explore) 
  
  - [**Package and File List**](#package-and-file-list)
  - [**Installation**](#installation)
  - [**Run**](#run)
  - [**System's Features**](#systems-features)
  - [**System's Limitation**](#systems-limitations)
  - [**Possible Technical Improvements**](#possible-technical-improvements)
  - [**Contacts**](#contacts)

## __Introduction__ 
This project represents the final assignment of the Experimental Robotics Laboratory corse. Therefore is an improvement of the assignment1 and assignment2 that you can find in this git repository as well. 

The aim of this assignment is to introduce some SLAM and autonomous navigation features. Infect the environment is now more complex and articulated, with colored balls representing the rooms of a hypothetical house, like shown in the following figure.

![Map](https://github.com/FraTesta/experimental_ws/blob/master/src/final_assignment/documentation/doc_pages/map.png)

The _NORMAL_ and _SLEEP_ mode maintain the previous behavior with the difference that when the robot is in the _NORMAL_ state and detects a ball, the robot will start to track it and store its own position after reaching it.
The user can now switch in the _PLAY_ mode typing the _play_ keyword. Then it can specify a desired room that the robot will reach (i.e. GoTo Bedroom). 

If the entered room hasn't yet been visited the robot will switch in the _FIND_ state where it should find the desired room.

## __Software Architecture__
For this project it was necessary to rely on already established and tested ROS packages, in particular as regards the navigation part (SLAM and autonomous navigation). 
The __gmapping__ algorithm was used to create the map of the environment and the __move base__ ROS package for the autonomous navigation part.

Those packages are very simple and therefore not too heavy in terms of computational load. Moreover taking into account the simplicity of the environment a 3D SLAM wouldn't have been so useful.

the software architecture implemented is shown below:

![SW architecture](https://github.com/FraTesta/experimental_ws/blob/master/src/final_assignment/documentation/doc_pages/rosgraph.png)

Notice that the architecture is "dynamic" in a sense that some topics are dynamically registered and deleted in order to improve the speed and the safety of the code.


### Description
- [commandManager](https://github.com/FraTesta/experimental_ws/blob/master/src/final_assignment/scripts/commandManager.py) = is the core of the architecture as in the previous assignments. It takes input from the _UI_ node and the _roomsDetector_ . Based on the state in which it is, this node can make requests to:
  -  the _move_base_ action server to reach a certain position.
  -  the _track_ action server to reach a detected room (ball).
- [roomDetector](https://github.com/FraTesta/experimental_ws/blob/master/src/final_assignment/scripts/roomDetector.py) = is a simple openCV algorithm that analyzes the camera images to detect the balls (rooms). After that this node sends the color of the detected ball to the _commandManager_. Finally it interrupts the subscription to the camera topic and goes in a sort of sleeping mode until the _commandManager_ awaken it again.
- [track](https://github.com/FraTesta/experimental_ws/blob/master/src/final_assignment/scripts/track.py) = is a action server that tracks a ball of a given color. The algorithm of tracking it's very similar to the _ball_track_ of the previous assignment. When the robot reaches the ball it will read its own position and send it back to the _commandManager_ so that it can store the position of the discovered room. If for some reason the ball is no longer detected the robot will turn on itself in both directions in an attempt to see the ball again. If after some time it has not succeeded, it switches back to the appropriate state.
Finally, it was implemented a very simple **obstacle_avoidance** algorithm using the laser scan data. It was necessary since when the robot starts to track a ball the _move_base_ algorithm is deactivated by the _commadManager_ and its integrated obstacle avoidance as well.
- [UI](https://github.com/FraTesta/experimental_ws/blob/master/src/final_assignment/scripts/UI.py) = is a very simple user interface that allows the user to switch in the _PLAY_ mode and choose a desired room to reach.
Notice that the _move_base_ goal is aborted every time a ball is detected or the play command is typed.
For more details regarding the scripts, see the doxygen documentation. 

### **Architecture Choices**
It's preferred to keep separate the _roomDetector_ and the _track_ nodes, even if quite similar, in order to get an asynchronous node (_roomDetector_) which notifies new rooms directly to the _commandManager_ which handle that information according to its state and its priority. Moreover this choice guaranties to implement the tracking phase as an action server and thus having access to all its features such as checking its status or aborting a mission. However the computational load is not increased since the roomDetector node goes into a kind of sleep mode when the _track_ is active. In this status the node doesn't perform any kind of image processing. This choice is due to the fact that the images processing of the _track_ and _roomDetector_ nodes are of course the most heavy computational load processes. 
 

### **ROS Messages and Actions**
The messages used for the custom topics are all belong to the _std_msgs_ library. In particular :
| Topic | Msg|
| ---- | ----- |
| **UIchatter** | uses a *String* message to send the user command input to the *commandManager* (*play* and *GoTo*). |
| **startRD** | which stands for start roomDetector, it's just a Bool message which enable/disable the ball detection.|
| **newRoom** | uses a String msgs to send the color of the detected room to the *commandManager*.|

While the action server is defined as follows:
```
# Goal
string color
---
# Result
float64 x
float64 y
---
# Feedback
string state
```
Basically it gets a color and returns the room position in terms of x and y coordinates.
## **System's Features** 
### **Move Base and Gmapping settings**
- The **_gmapping_** parameters are contained in the [gmapping.launch](https://github.com/FraTesta/experimental_ws/blob/master/src/final_assignment/launch/gmapping.launch) file. Basically I changed:
  - The **maxUrange** parameter (increased 20) to increase the range of the laser in order to map deeper the environment, since in the biggest room the robot didn't detect any wall, this introduced a localization error. 
  - The **lsigma** and **ogain** parameter for smoothing the resampling effects which was too hard.
  - Increased the **particles** parameter to increase the ability of the robot to close the loop.
- Regarding the **_move_base_** settings:
  - In the [base_local_planner_params.yaml](https://github.com/FraTesta/experimental_ws/blob/master/src/final_assignment/param/base_local_planner_params.yaml) I increased the **max_vel_x**, **min_vel_x**, **acc_lim_x** and **acc_lim_theta** params to make the robot faster and more responsive. Finally I increased the **sim_time** in order to improve the local planning simulation because some times the chosen trajectory was not consistent with the environment and the global path trajectory. 
  - In the [costmap_common_params.yaml](https://github.com/FraTesta/experimental_ws/blob/master/src/final_assignment/param/costmap_common_params.yaml) I increased the **obstacle_range** and the **robot_radius** in order to keep the robot away from walls, especially near corners or entrances.
  - In the [global_costmap_params.yaml](https://github.com/FraTesta/experimental_ws/blob/master/src/final_assignment/param/global_costmap_params.yaml) I increased the **update_frequency** and **publish_frequency** to make the planner more reactive to changes and faster in correcting mapping errors. I increased also the _inflation_radius_ to make shore that the robots enter every room.
  - Finally in the [local_cost_map.yaml](https://github.com/FraTesta/experimental_ws/blob/master/src/final_assignment/param/localcostmap_params.yaml) I increased tge **with** and **height** parameters to improve the local mapping and avoid strange trajectory.

### __FSM Description__
The structure of the finite state machine is inevitably more complicated like shown in the figure. The FSM was still developed using the _smach_ API so you can still use its feature to study this new implementation.

![FSM](https://raw.githubusercontent.com/FraTesta/experimental_ws/master/src/final_assignment/documentation/doc_pages/FSM.png)

The states are still implemented in the [commandManager.py](https://github.com/FraTesta/experimental_ws/blob/master/src/final_assignment/scripts/commandManager.py) script which now receives input from the user interface ([UI.py](https://github.com/FraTesta/experimental_ws/blob/master/src/final_assignment/scripts/UI.py) script) and from the rooms detector node ([roomsDetector.py](https://github.com/FraTesta/experimental_ws/blob/master/src/final_assignment/scripts/roomsDetector.py) script) that change the robot states using their custom topics. 

| State | Description |
| ---| ---|
| NORMAL | Is the initial state and gives some random goals to the _move_base_. At each iteration checks if a new ball is detected (Switch in the _TRACK_ state) or if the user typed the 'play' keyword (switch in the  _PLAY_ state). After some iteration it switches in the *SLEEP* state. |
| PLAY | the robot returns to the initial position (Home) and in the meantime checks if the user types a 'GoTo' command. If the entered room is in the roomStructure it will reach it and returns home. Otherwise switches in the _FIND_ state. |
| TRACK | Starts when the _commandManager_ receives the color of a new detected ball. Thus it makes a request to the _track_ action server to track the ball. When the action server has finished this state it saves the returned location and associates it with the correct room. Then it returns to the appropriate state. For instance if the previous state was _FIND_ it checks if the detected ball is the desired one. If so switches to the _PLAY_ state otherwise switches back to the _FIND_. |
| FIND | Starts to explore the environment using the [explore](#explore) function of the [Rooms](https://github.com/FraTesta/experimental_ws/blob/master/src/final_assignment/scripts/Rooms.py) class. At each iteration it checks if a new ball is detected, if so it will switch momentarily in the _TRACK_ state. After some iterations it switch back to the _PLAY_ state. |
| SLEEP | The robot goes to the home position and stay there for some time after that switches back to the _NORMAL_ state.

### **Robot Model & Knowledge Rappresentation** 
The robot model used is the same of the previous assignment but I removed the neck joint in order to keep the head fixed and thus the camera as well. Moreover I added a laser sensor which is necessary for the *gmapping* and *move_base* algorithms. 

![RoboModel](https://github.com/FraTesta/experimental_ws/blob/master/src/final_assignment/documentation/doc_pages/robotModel.jpg)

You can see the robot simulation and the environment using:
```
roslaunch final_assignment simulation.launch
```

Regarding the knowledge representation I develop a class called _Rooms.py_ that provides a simple structure that associates each room with a color ball and their respecting location in the space in terms of x and y coordinates as shown below:
 ```
 self.ROOMS = [ 
        {'name':"Entrance",'color': "blue", "x":0, "y":0, 'detected':False},
        {'name':"Closet",'color': "red", "x":0, "y":0, 'detected':False},
        {'name':"Leavingroom",'color': "green", "x":0, "y":0, 'detected':False},
        {'name':"Kitchen",'color': "yellow", "x":0, "y":0, 'detected':False},
        {'name':"Bathroom",'color': "magenta", "x":0, "y":0, 'detected':False},
        {'name':"Bedroom",'color':"black","x":0,"y":0, 'detected':False},
        {'name':"Home",'color':"","x": -5,"y":7, 'detected':True}
        ]
```
Of course this class provides also methods to update the knowledge of the environment, get information... For more information take a look to its [doxygen documentation](file:///home/francescotesta/experimental_ws/src/final_assignment/documentation/html/classRooms_1_1Rooms.html) .

### **Explore**
The explore function of the class [Rooms](https://github.com/FraTesta/experimental_ws/blob/master/src/final_assignment/scripts/Rooms.py) has nothing to do with the _explore-lite_ package. It is a function that generates random goal positions considering the already visited room locations that are stored in the Rooms dictionary. Basically, it draws a virtual area of 2x2 meters (the area dimension can be editable) around any stored room location and discards every goal generated within them. Then it applies the same concept to every location previously visited, during the Find mode, that were stored in a list. This is to avoid revisiting the same places over and over.

This solution improves its effectiveness as rooms are discovered. However it has the drawback that works pretty fine in this particular environment where the balls are sufficiently distant, it may not be so good in other contexts.

## **Package and File List**
The final assignment package provides the following directory:
- **action** = contains the definition of the _track_ action server
- **config** = contains some setting for the RViz simulation 
- **launch** = contains the some launch files that are better shown in the [Run](#run) section.
- **param** = contains some configuration file.yaml of the _move_base_ package (see [Move Base and Gmapping settings](#move-base-and-gmapping-settings))
- **scripts** = contains the code (see [SW architecture](#software-architecture)) 
- **urdf** = contains the urdf models files 
- **world** = the world gazebo simulation 

## **Installation**
This project might have some problems if it runs on a docker machine, thus a virtual machine or a clean version of Ubuntu 16.04 is preferred. 

The packages used for the previous assignments are still needed.
Download the *gmapping* and *move_base* packages, I suggest to clone the following repository which were used in the project: 
- **gmapping =**  https://github.com/CarmineD8/SLAM_packages.git
- **move_base =** https://github.com/CarmineD8/planning.git
Install also:
```
sudo apt-get install ros-kinetic-openslam-gmapping
sudo apt-get install ros-kinetic-navigation
```
Then build the workspace.

## **Run**
First of all as always source the workspace. I provide different launch files and parameters such as:
- **ui** = if set to true it will launch the UI node to interact with the project.
- **rviz** = open the Rviz simulation (_rviz:=true_).

Launch the project (with the User Interface)
```
roslaunch final_assignment start.launch ui:=true
```
Now you should see two terminal windows: 
One for the User Interface 
```
******************************
 Welcome !!!! 

 The user can digit the keyword: 
 - 'play' -> to switch in play mode 
 - 'GoTo roomName' -> to reach that room or to start looking for it (if it hasn't yet been discovered) 
 - 'list' -> to get the list of the rooms already visited 
 The rooms present are:
 Entrance(blue)
 Closet(red)
 Leavingroom(green)
 Kitchen(yellow)
 Bathroom(Magenta)
 Bedroom(black)
*******************************
```
That allows you to interact with the robot simulation. Notice that you can also type 'list' to get a list of the already visited rooms.

 While the second window shows some information about the robot and its behaviors.
```
[INFO] [1619879582.422315, 429.745000]: State machine starting in initial state 'NORMAL' with userdata: 
	[]
[INFO] [1619879582.422729, 429.745000]: ***********************************
[INFO] [1619879582.423006, 429.745000]: [CommandManager] I m in NORMAL state
[INFO] [1619879584.425615, 430.692000]: 0
[INFO] [1619879584.425931, 430.692000]: [CommandManager] generate a new random goal position
[INFO] [1619879584.426230, 430.692000]: [CommandManager] I'm going to position x = 0 y = -5
[INFO] [1619879585.642455, 431.083000]: [CommandManager] reach a new ball of color blue

```
Notice that you can enter a desired room (using the _GoTo_ command) at any time, but it will be executed only in the _PLAY_ state (so after typing the _play_ command).
Finally you can run the project without the _SLEEP_ mode which pretty useful for debugging:
```
roslaunch final_assignment noSleep.launch
```
You can check the track phase and its obstacle avoidance algorithm by running this test version:
```
roslaunch final_assignment testTrackObsAv.launch
```

## **System's Limitations**
**a)**  The system was tested for a long period having in general good behaviors. However some times in the first terminal might appear an error message saying that the robot is not able to find a global path. It's a problem related to the *move_base* package settings. I was not able to solve it, however it's very rare that happens and most of the time the system automatically solves such problem by finding again the path or detecting a new ball. If for this reason the robot stops, it is sufficient for the user to type "play" to abort the current mission. 

**b)**  Another possible limitation is the _explore_ function that in some cases requires a few attempts to find a certain ball in the begging phase and it might not work so good with other environments as already explained in the [Explore description](#explore). 

**c)**  Finally the obstacle avoidance algorithm doesn't work properly in every context. Infect sometime it may choose a wrong direction to approach an obstacle. Moreover it could happen that the robot collides with an obstacle with the wheels since the algorithm doesn't taking into account the obstacle on the left and right ranges (but only front, front-right, front-left).

## **Possible technical improvements** 


1. Develop a more sophisticated Knowledge representation, maybe designing an Ontology with an editors or API like Protegè.
2. Improve the explore function maybe integrating it with some other exploration algorithm like Bug2 or explore-lite. 

3. A better _move_base_/_gmapping_ parameters could be found since currently are set in order to avoid robot collisions or blocking problems. But with other settings it could do fewer maneuvers and smoother trajectory.
4. Improve the obstacle avoidance algorithm of the _track_ node. Since actually the robot is only able to avoid obstacle that are in front of it. 

## **Contacts**
Francesco Testa

francesco.testa.ge@gmail.com




 

