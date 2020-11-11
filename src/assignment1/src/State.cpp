
/*!
 * \author Francesco Testa
 * \version 1.0
 * \date 10-27-2020
 * \mainpage ASSIGNMENT 1
 * \section Introduction
 * This code implements a basic software architecture in according to my first assignment of the experimental robotics course.
 * If you want to learn more about code development please read the README file in the git repository
*/

/*!
 * \section Description
 * This code generates a ROS publisher node which publishes a string 
 * 'play', using a std_msgs/String, on the topic SateString each iteration which is defined by the time variable
 * @param time represents the waiting time before generating another 'play' message that is randomly generated at each iteration to simulate a user 
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main (int argc, char **argv){

ros::init(argc,argv,"speakPerception"); 

ros::Publisher pub;
ros::NodeHandle nh;

pub = nh.advertise<std_msgs::String>("StateString", 100); 

ros::Rate loop_rate(1); 

int time = 15; 

while(ros::ok()){

std_msgs::String msg; 

std::stringstream ss;

    msg.data = "play"; 

    ROS_INFO("state detection: %s", msg.data.c_str());

    pub.publish(msg);

    ros::spinOnce();

    sleep(time);
    time = rand() % 8 + 15; 


}

return 0;
}