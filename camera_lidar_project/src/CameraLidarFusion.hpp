#pragma once

#include <ros/ros.h>
#include <avs_lecture_msgs/TrackedObjectArray.h>
#include <sensor_msgs/Image.h>
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
#include <darknet_ros_msgs/BoundingBox.h>
#include <math.h>
#include <visualization_msgs/MarkerArray.h>


namespace camera_lidar_project
{

class CameraLidarFusion
{
  public:
    CameraLidarFusion(ros::NodeHandle n, ros::NodeHandle pn);

  private:
    void reconfig(CameraLidarFusionConfig& config, uint32_t level);
    void recvImage(const sensor_msgs::ImageConstPtr& msg);
    void recvCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg);
    void recvLidarObjects(const avs_lecture_msgs::TrackedObjectArrayConstPtr& msg);
    void recvDetectionImage(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);
    cv::Rect2d getCamBbox(const avs_lecture_msgs::TrackedObject& object, const tf2::Transform& transform, const image_geometry::PinholeCameraModel& model);
    bool IoU(cv::Rect2d r1,cv::Rect2d r2);

    //revisit
    //void generateBoundingBoxes(const avs_lecture_msgs::TrackedObject& object);

    geometry_msgs::Point projectPoint(const image_geometry::PinholeCameraModel& model, const cv::Point2d& p);

    tf2_ros::TransformListener listener_;
    tf2_ros::Buffer buffer_;
    tf2_ros::StaticTransformBroadcaster broadcaster_;

    ros::Subscriber sub_image_;
    ros::Subscriber sub_cam_info_;
    ros::Subscriber sub_detections_;
    ros::Subscriber sub_lidar_objects_;
    ros::Publisher pub_markers_;
    //ros::Publisher pub_bboxes_;
    ros::Publisher label_object;
    std::shared_ptr<dynamic_reconfigure::Server<CameraLidarFusionConfig> > srv_;

    sensor_msgs::CameraInfo camera_info_;
    geometry_msgs::TransformStamped camera_transform_; // Coordinate transformation from footprint to camera
    bool looked_up_camera_transform_;
    std::vector<cv::Rect2d> cam_bboxes_;
    std::vector<darknet_ros_msgs::BoundingBox> detections;

    //revisit
    //std::vector<avs_lecture_msgs::TrackedObjectArray_> car_boxes;
    //avs_lecture_msgs::TrackedObjectArray car_boxes;

    bool doOverlap;
    uint32_t bbox_id;
    uint32_t car_boxid;
    _Float64 bbox_scale_x; 
    _Float64 bbox_scale_y; 
    _Float64 bbox_scale_z; 
    _Float64 bbox_pos_x; 
    _Float64 bbox_pos_y; 
    _Float64 bbox_pos_z; 
    _Float64 bbox_orientation;

    
};

}
