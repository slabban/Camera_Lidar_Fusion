// ROS and node class header file
#include <ros/ros.h>
#include "Homework4.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "homework4");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  homework4::Homework4 node(n, pn);

  // Spin and process callbacks
  ros::spin();
}
