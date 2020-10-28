#include "ros/ros.h"
#include "assignment1/GoTo.h"

// max dimension for the map
#define Xmax 20
#define Ymax 20 

bool goTo(assignment1::GoTo::Request  &req,
         assignment1::GoTo::Response &res)
{
  ROS_INFO("Go to position: x=%ld, y=%ld", (long int)req.x, (long int)req.y);
  if((req.x <= Xmax) && (req.y <= Ymax)){
    res.o = 0; // the location is within the map boundaries
  }else{
        res.o = 1; // it's not
       }
  
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