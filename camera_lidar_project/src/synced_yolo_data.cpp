// ROS and node class header file
#include <ros/ros.h>
#include "SyncedYoloData.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "synced_yolo_data");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  camera_lidar_project::SyncedYoloData node(n, pn);

  // Spin and process callbacks
  ros::spin();
}
