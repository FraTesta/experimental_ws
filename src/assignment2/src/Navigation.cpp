 /*!
 * @file Navigation.cpp
 * @section Description
 * \brief It simulate the navigation from the current position to the position requested by the client, taking into account the dimension of a map a priori chosen 
 * \subsection input/output
 * This service takes a X and Y positon request from the "command manager" node which is the only client.
 * \see commandManager.py
 */

#include "ros/ros.h"
#include "assignment2/GoTo.h"

/// Library necessary to use Twists ROS messages
#include "geometry_msgs/Twist.h"
/// Library necessary to use Odometry ROS messages
#include "nav_msgs/Odometry.h"

#include <iostream>



/// Max dimension of the map along the X axis 
#define Xmax 20
/// Max dimension of the map along the Y axis 
#define Ymax 20 
/// X position of the House
#define homeX 10 
/// Y position of the House 
#define homeY 20 
/// X position of the user
#define userX 2 
/// Y position of the user
#define userY 3 

ros::Publisher pub;

void printlogs(int x ,int y){
  ///  \section printlogs
  /// It's a simple function that, based on the requested x y position, generates a log in the terminal
  if((x == homeX)&&(y == homeY)){
     ROS_INFO(" The robot is going home, wait...");
    }else if ((x == userX)&&(y == userY)){
      ROS_INFO(" The robot is returning to the user, wait...");
    }else{ROS_INFO(" It's going into position: x=%ld, y=%ld wait...",x,y);
    }
}


bool goTo(assignment2::GoTo::Request  &req,
         assignment2::GoTo::Response &res)
{ 
  /*!
* \section goTo
* This is the serve function which menages the requests from clients. 
* It checks if the target position is within the map predefined by the parameters Xmax and Ymax
* if it is inside then it simulates the navigation simply waiting 3 seconds after which it assigns the request values to the response and sets to true the check variable.
* Otherwise it assigns false to the check variable 
*/ 
  geometry_msgs::Twist vel;
        
  printlogs((long int)req.x,(long int)req.y);
  if((req.x <= Xmax) && (req.y <= Ymax)){
     sleep(3); //which simulate the movement of the robot 
     res.ok = true;
     
     vel.linear.x = req.x;
     vel.angular.z= req.y;
     pub.publish(vel);

     res.currentX = req.x;
     res.currentY = req.y; 
      }else{
        res.ok = false; 
       }
  
  return true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "Navigation_server"); //node init
  ros::NodeHandle n;
  /// @param pub Publisher to send velocity commands on the topic cmd_vel
  pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  
  ros::ServiceServer service = n.advertiseService("myNavigation", goTo);
  ROS_INFO(" Ready to go in a new position.");
  ros::spin();

  return 0;
}
