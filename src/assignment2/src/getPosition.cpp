/*!
 * @file getPosition.cpp 
 * @section Description
 * \brief This node publishes a random position which will be used from the command manager node .
 * In practice it simulates the pointing gesture from the user and the random travel in NORMAL mode as well.
 * @param x is an integer which represents the X position that is randomly chosen at each iteration.
 * @param y is an integer which represents the Y position that is randomly chosen at each iteration
 * @param pose is a geometry_msgs/Pose2D message in which we put the two previous values
 * \see commandManager.py
 */


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

int main (int argc, char **argv){


ros::init(argc,argv,"getPosition"); 

ros::Publisher pub;
ros::NodeHandle nh;

//pub = nh.advertise<geometry_msgs::Pose2D>("Position", 1000); //pub and topic init 
pub = nh.advertise<geometry_msgs::Twist>("Random_vel", 5);

ros::Rate loop_rate(1); // that I m going to make it randomly in the simulation phase 


while(ros::ok()){

    geometry_msgs::Twist vel;
    vel.linear.x = randMToN(-0.2, 0.2);
    vel.angular.z= randMToN(-0.2, 0.2);
    pub.publish(vel);


    ROS_INFO("The vel is : %d and the angular vel is: %d ", vel.linear.x, vel.angular.z);

    ros::spinOnce();
    sleep(100);
    loop_rate.sleep();
   

}

return 0;
}
