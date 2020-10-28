#include "ros/ros.h"
//#include "assignment1/State.h"
#include "std_msgs/String.h"
#include <sstream>

int main (int argc, char **argv){

ros::init(argc,argv,"speakPerception"); //node init

ros::Publisher pub;
ros::NodeHandle nh;

pub = nh.advertise<std_msgs::String>("StateString", 100); //pub and topic init 

ros::Rate loop_rate(8); // that I m going to make it randomly in the simulation phase 
int count = 0; 
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */

while(ros::ok()){

std_msgs::String msg; 

std::stringstream ss;
    ss << "play " << count;
    msg.data = ss.str();

    ROS_INFO("state detection: %s", msg.data.c_str());

    pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
}

return 0;
}