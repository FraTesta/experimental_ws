/*!
 * \section Description
 * This node publishes a random position which will be used from the command manager node.
 * In practice it simulates the pointing gesture from the user and the random travel in NORMAL mode as well.
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>



int main (int argc, char **argv){

    
    int x = rand() % 15; 
    int y = rand() % 15; 



ros::init(argc,argv,"getPosition"); 

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

}

return 0;
}