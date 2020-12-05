#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <avs_lecture_msgs/TrackedObjectArray.h>
#include <camera_lidar_project/CameraLidarFusionConfig.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <image_geometry/pinhole_camera_model.h>
#include <dynamic_reconfigure/server.h>
#include <camera_lidar_project/CameraLidarFusionConfig.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <math.h>
#include <visualization_msgs/MarkerArray.h>
#include <camera_lidar_project/FusedObjectArray.h>

// Filters
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace cv;
//changed
namespace camera_lidar_project
{
  // Exact Time Syncronizer for Image Messages and Yolo 
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> YoloSyncPolicy;

  // Approximate Time Syncronizer for Lidar and sensor messages
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, avs_lecture_msgs::TrackedObjectArray> LidarSyncPolicy;

  // DISREGARD
  //typedef message_filters::sync_policies::ExactTime<darknet_ros_msgs::BoundingBoxes, avs_lecture_msgs::TrackedObjectArray> LidarSyncPolicy;



  class SyncedYoloData
  {
    public:
      SyncedYoloData(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:

      void reconfig(CameraLidarFusionConfig& config, uint32_t level);

      void recvCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg);

      void recvSyncedData(const sensor_msgs::ImageConstPtr& img_msg, const darknet_ros_msgs::BoundingBoxesConstPtr& bbox_msg);
      
      void recvLidarSynced(const sensor_msgs::ImageConstPtr& img_msg, const avs_lecture_msgs::TrackedObjectArrayConstPtr& object_msg);
      //void recvLidarSynced(const darknet_ros_msgs::BoundingBoxesConstPtr& bbox_msg, const avs_lecture_msgs::TrackedObjectArrayConstPtr& object_msg);
      void recvLidarObjects(const avs_lecture_msgs::TrackedObjectArrayConstPtr& msg);

      cv::Rect2d getCamBbox(const avs_lecture_msgs::TrackedObject& object, const tf2::Transform& transform, const image_geometry::PinholeCameraModel& model);

      bool IoU(cv::Rect2d r1, const darknet_ros_msgs::BoundingBox& detect,  int stale_objects);

      //void updateTimerCallback(const ros::TimerEvent& event);


      //Subscriber and Sychronizer for image, darknet bounding boxes, and LIDAR
      boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > sub_img_;
      boost::shared_ptr<message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> > sub_objects_;
      boost::shared_ptr<message_filters::Subscriber<avs_lecture_msgs::TrackedObjectArray> > sub_lidar_;
      boost::shared_ptr<message_filters::Synchronizer<YoloSyncPolicy> > sync_yolo_data_;
      boost::shared_ptr<message_filters::Synchronizer<LidarSyncPolicy> > sync_lidar_data_;

      std::shared_ptr<dynamic_reconfigure::Server<CameraLidarFusionConfig> > srv_;

      geometry_msgs::Point projectPoint(const image_geometry::PinholeCameraModel& model, const cv::Point2d& p);

      tf2_ros::TransformListener listener_;
      tf2_ros::Buffer buffer_;
      tf2_ros::StaticTransformBroadcaster broadcaster_;

      ros::Subscriber sub_cam_info_;
      ros::Publisher car_bboxes_;
      ros::Subscriber sub_lidar_objects_;
      ros::Timer update_timer_;



    sensor_msgs::CameraInfo camera_info_;
    geometry_msgs::TransformStamped camera_transform_; // Coordinate transformation from footprint to camera
    bool looked_up_camera_transform_;

    std::vector<cv::Rect2d> cam_bboxes_;

    std::vector<std::pair<cv::Rect2d, avs_lecture_msgs::TrackedObject> > myArr;
    

    avs_lecture_msgs::TrackedObjectArray car_boxes;
    //avs_lecture_msgs::TrackedObjectArray final_boxes;

    bool doOverlap;

    //bool similar_box;

    std::vector<int> previous_Box;

    std::vector<int> One_run;

    

    //ros::Time estimate_stamp_;

    

  };
}