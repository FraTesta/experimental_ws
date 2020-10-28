#include "ros/ros.h"
//#include "assignment1/State.h"
#include "std_msgs/String.h"
#include <sstream>

int main (int argc, char **argv){

ros::init(argc,argv,"speakPerception"); //node init

ros::Publisher pub;
ros::NodeHandle nh;

pub = nh.advertise<std_msgs::String>("StateString", 100); //pub and topic init 

ros::Rate loop_rate(1); // that I m going to make it randomly in the simulation phase 

int time = 15;

while(ros::ok()){

std_msgs::String msg; 

std::stringstream ss;
    //ss << "play" << count;
    //msg.data = ss.str();
    msg.data = "play";

    ROS_INFO("state detection: %s", msg.data.c_str());

    pub.publish(msg);

    ros::spinOnce();

    sleep(time);
    time = rand() % 8 + 13;


}

return 0;
}