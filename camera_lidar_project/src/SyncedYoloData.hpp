#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

// Filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

//changed
namespace camera_lidar_project
{

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> YoloSyncPolicy;

  //Create Approximate time here for Lidar and Yolo detections

  //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> YoloSyncPolicy;

  class SyncedYoloData
  {
    public:
      SyncedYoloData(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:

      void recvSyncedData(const sensor_msgs::ImageConstPtr& img_msg, const darknet_ros_msgs::BoundingBoxesConstPtr& object_msg);

      boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > sub_img_;
      boost::shared_ptr<message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> > sub_objects_;
      boost::shared_ptr<message_filters::Synchronizer<YoloSyncPolicy> > sync_yolo_data_;
  };
}