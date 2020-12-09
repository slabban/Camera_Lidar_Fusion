// ROS and node class header file
#include <ros/ros.h>
#include "Lidar_ekf.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "lidar_ekf");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  lidar_ekf::Lidar_ekf node(n, pn);

  // Spin and process callbacks
  ros::spin();
}
