#include "SyncedYoloData.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

//changed
namespace camera_lidar_project
{

  // /camera/image_rect_color

  SyncedYoloData::SyncedYoloData(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    sub_img_.reset(new message_filters::Subscriber<sensor_msgs::Image>(n, "/darknet_ros/detection_image", 5));
    sub_objects_.reset(new message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>(n, "/darknet_ros/bounding_boxes", 5));
    sync_yolo_data_.reset(new message_filters::Synchronizer<YoloSyncPolicy>(YoloSyncPolicy(10), *sub_img_, *sub_objects_));
    sync_yolo_data_->registerCallback(boost::bind(&SyncedYoloData::recvSyncedData, this, _1, _2));

    namedWindow("Sync_Output", WINDOW_NORMAL);
  }

  void SyncedYoloData::recvSyncedData(const sensor_msgs::ImageConstPtr& img_msg, const darknet_ros_msgs::BoundingBoxesConstPtr& object_msg)
  {
    
    Mat img_raw = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;

    for (auto& bbox : object_msg->bounding_boxes) {

      double width = (bbox.xmax-bbox.xmin);
      double height =(bbox.ymax-bbox.ymin);

      cv::Point2d corner(bbox.xmin, bbox.ymin);
      rectangle(img_raw, cv::Rect(bbox.xmin, bbox.ymin, width, height), Scalar(0, 255, 0));
      //putText(raw_img, bbox.label, corner, FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 255, 255));
    }

    imshow("Sync_Output", img_raw);
    waitKey(1);
    
  // create a global message and an apporximate sycronizer for the Lidar and Camera info, implement the IOU algorithm there!

  }

}