#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

/*
This node publish random position which will be used from the command manager node.
This node simulates the pointing gesture and the random travel in normal mode as well.

*/

int main (int argc, char **argv){

//generate a random position which simulates the pointin gestiure and the travel in normal mode 
int x = rand() % 15;
int y = rand() % 15;

//int time = rand() % 5000 + 1000;


ros::init(argc,argv,"getPosition"); //node init

ros::Publisher pub;
ros::NodeHandle nh;

pub = nh.advertise<geometry_msgs::Twist>("Position", 1000); //pub and topic init 

ros::Rate loop_rate(1); // that I m going to make it randomly in the simulation phase 


while(ros::ok()){

geometry_msgs::Twist vel; 
    
    vel.linear.x = x ;
    vel.linear.y = y ;


    ROS_INFO("The position is x : %d and y : %d ", x,y);

    pub.publish(vel);

    ros::spinOnce();

    loop_rate.sleep();
    x = rand() % 15;
    y = rand() % 15;
    //sleep(time);
    //time = rand() % 5000 + 1000;
}

return 0;
}