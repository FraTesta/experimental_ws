

#include "ros/ros.h"
#include "assignment1/GoTo.h"



#define Xmax 20 //!< Max dimension of the map along the X axis 
#define Ymax 20 //!< Max dimension of the map along the Y axis 


bool goTo(assignment1::GoTo::Request  &req,
         assignment1::GoTo::Response &res)
{
  ROS_INFO(" It's going into position: x=%ld, y=%ld wait...", (long int)req.x, (long int)req.y);
  if((req.x <= Xmax) && (req.y <= Ymax)){
     sleep(3); //which simulate the movement of the robot 
     res.ok = true;
     res.currentX = req.x;
     res.currentY = req.y; 
      }else{
        res.ok = false; 
       }
  
  return true;
}

int main(int argc, char **argv)
{
   /*!
 * \section Description
 * It simulate the navigation from the current position to the position requested by the client, taking into account the dimension of a map a priori chosen 
 * \subsection INPUT / OUTPUT
 * This service takes a X and Y positon request from the "command manager" node which is the only client.
 */
  ros::init(argc, argv, "Navigation_server"); //node init
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("myNavigation", goTo);
  ROS_INFO(" Ready to go in a new position.");
  ros::spin();

  return 0;
}