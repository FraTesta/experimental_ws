#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <sstream>

int main (int argc, char **argv){

ros::init(argc,argv,"getPosition"); //node init

ros::Publisher pub;
ros::NodeHandle nh;

pub = nh.advertise<geometry_msgs::Twist>("Position", 100); //pub and topic init 

ros::Rate loop_rate(3); // that I m going to make it randomly in the simulation phase 
int count = 0; 
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */

while(ros::ok()){

geometry_msgs::Twist vel; 

    vel.linear.x = 5;
    vel.linear.y = 8;


    ROS_INFO("The X linear position is  : %d", vel.linear.x);

    pub.publish(vel);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
}

return 0;
}