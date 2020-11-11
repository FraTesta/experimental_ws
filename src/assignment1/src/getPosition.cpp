/*!
 * \section Main_Description
 * This node publishes a random position which will be used from the command manager node.
 * In practice it simulates the pointing gesture from the user and the random travel in NORMAL mode as well.
 * @param x is an integer which represents the X position that is randomly chosen at each iteration.
 * @param y is an integer which represents the Y position that is randomly chosen at each iteration
 * @param pose is a geometry_msgs/Pose2D message in which we put the two previous values
 */


#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

int main (int argc, char **argv){

    
    int x = rand() % 15; 
    int y = rand() % 15; 



ros::init(argc,argv,"getPosition"); 

ros::Publisher pub;
ros::NodeHandle nh;

pub = nh.advertise<geometry_msgs::Pose2D>("Position", 1000); //pub and topic init 

ros::Rate loop_rate(1); // that I m going to make it randomly in the simulation phase 


while(ros::ok()){

geometry_msgs::Pose2D pose; 
    
    pose.x = x ;
    pose.y = y ;


    ROS_INFO("The position is x : %d and y : %d ", x,y);

    pub.publish(pose);

    ros::spinOnce();

    loop_rate.sleep();
   
    x = rand() % 15;
    y = rand() % 15;

}

return 0;
}