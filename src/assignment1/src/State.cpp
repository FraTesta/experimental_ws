
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
 * This code generates a ROS node which is also a publisher. It publishes a string 
 * 'play' on the topic SateString. 
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

int time = 15; /// initialization of the variable which will randomly choose how often to generate a 'play' message

while(ros::ok()){

std_msgs::String msg; 

std::stringstream ss;

    msg.data = "play"; 

    ROS_INFO("state detection: %s", msg.data.c_str());

    pub.publish(msg);

    ros::spinOnce();

    sleep(time);
    time = rand() % 8 + 15; /// choose randomly the next time iteration


}

return 0;
}