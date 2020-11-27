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

//changed
namespace camera_lidar_project
{

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> YoloSyncPolicy;

  //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, avs_lecture_msgs::TrackedObjectArray> LidarSyncPolicy;
  typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, avs_lecture_msgs::TrackedObjectArray> LidarSyncPolicy;

  //Create Approximate time here for Lidar and Yolo detections

  //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> YoloSyncPolicy;

  class SyncedYoloData
  {
    public:
      SyncedYoloData(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:

      void reconfig(CameraLidarFusionConfig& config, uint32_t level);

      void recvCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg);

      void recvSyncedData(const sensor_msgs::ImageConstPtr& img_msg, const darknet_ros_msgs::BoundingBoxesConstPtr& bbox_msg);
      
      //void recvLidarSynced(const sensor_msgs::ImageConstPtr& img_msg, const avs_lecture_msgs::TrackedObjectArrayConstPtr& object_msg);
      void recvLidarSynced(const darknet_ros_msgs::BoundingBoxesConstPtr& bbox_msg, const avs_lecture_msgs::TrackedObjectArrayConstPtr& object_msg);

      cv::Rect2d getCamBbox(const avs_lecture_msgs::TrackedObject& object, const tf2::Transform& transform, const image_geometry::PinholeCameraModel& model);

      bool IoU(cv::Rect2d r1, const darknet_ros_msgs::BoundingBox& detect);

      //Subscriber and Sychronizer for image and darknet bounding boxes
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
      /*
      ros::Subscriber sub_image_;
      ros::Subscriber sub_detections_;
      ros::Subscriber sub_lidar_objects_;
      ros::Publisher pub_markers_;
      //ros::Publisher pub_bboxes_;
      ros::Publisher car_bboxes_;
      ros::Publisher bounding_boxes_;
      ros::Publisher detectionImagePublisher_;
      */


    sensor_msgs::CameraInfo camera_info_;
    geometry_msgs::TransformStamped camera_transform_; // Coordinate transformation from footprint to camera
    bool looked_up_camera_transform_;
    std::vector<cv::Rect2d> cam_bboxes_;
    std::vector<darknet_ros_msgs::BoundingBox> detections;
    

    //revisit
    //FusedObjectArray car_boxes;
    avs_lecture_msgs::TrackedObjectArray car_boxes;

    bool doOverlap;
    uint32_t bbox_id;
    uint32_t car_boxid;
    //std_msgs::Header bbox_header; 
    _Float64 bbox_scale_x; 
    _Float64 bbox_scale_y; 
    _Float64 bbox_scale_z; 
    _Float64 bbox_pos_x; 
    _Float64 bbox_pos_y; 
    _Float64 bbox_pos_z; 
    _Float64 bbox_orientation;
  };
}