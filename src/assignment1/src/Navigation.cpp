#include "ros/ros.h"
#include "assignment1/GoTo.h"

bool goTo(assignment1::GoTo::Request  &req,
         assignment1::GoTo::Response &res)
{
  ROS_INFO("Go to quest: x=%ld, y=%ld", (long int)req.x, (long int)req.y);
  res.o = 1;
  
  //ROS_INFO("sending back response x: [%ld]", (char *)res.resp);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Navigation_server"); //node init
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("myNavigation", goTo);
  ROS_INFO("Ready to go in a new position.");
  ros::spin();

  return 0;
}