#include "CameraLidarFusion.hpp"

using namespace cv;

#define CALIBRATE_ALIGNMENT 0

namespace camera_lidar_project
{

CameraLidarFusion::CameraLidarFusion(ros::NodeHandle n, ros::NodeHandle pn) :
  listener_(buffer_)
{
  ros::NodeHandle cam_nh("camera");
  sub_cam_info_ = cam_nh.subscribe("camera_info", 1, &CameraLidarFusion::recvCameraInfo, this);
  sub_image_ = cam_nh.subscribe("image_rect_color", 1, &CameraLidarFusion::recvImage, this);
  sub_lidar_objects_ = n.subscribe("detected_objects", 1, &CameraLidarFusion::recvLidarObjects, this);
  sub_detections_ = n.subscribe("/darknet_ros/bounding_boxes", 1, &CameraLidarFusion::recvDetectionImage, this);

  looked_up_camera_transform_ = false;
#if CALIBRATE_ALIGNMENT
  srv_.reset(new dynamic_reconfigure::Server<CameraLidarFusionConfig>);
  srv_->setCallback(boost::bind(&CameraLidarFusion::reconfig, this, _1, _2));
#endif
  namedWindow("Output", cv::WINDOW_NORMAL);
  // namedWindow("First Object", cv::WINDOW_NORMAL);

  
}

void CameraLidarFusion::recvDetectionImage(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{

  detections = msg->bounding_boxes ;

  //darknet_ros_msgs::BoundingBox yolo_bbox;

  //double bbox_width = yolo_bbox.xmax;

  //ROS_INFO("xmin: %i", bbox_width);
  
/*
for (auto& detect : detections)
{

  //darknet_ros_msgs::BoundingBox yolo_bbox;

  //double bbox_width = yolo_bbox.xmax;

  //ROS_INFO("xmin: %i", yolo_bbox.xmax);


//Alternatively could use 
  if (detect.Class != "car"){

    continue;

  }

//ROS_INFO("%s \n", detect.Class.c_str());

float width = 0.5*(detect.xmin+detect.xmax);
float height = 0.5 * (detect.ymin+detect.ymax);

//ROS_INFO("Image width: %f \n", width);

//ROS_INFO("Image height: %f \n", height);


}*/



}

// This function is called whenever a new image is received from either
// the live running camera driver, a bag file recording, or the simulated
// image coming from Gazebo
void CameraLidarFusion::recvImage(const sensor_msgs::ImageConstPtr& msg)
{
  // Convert ROS image message into an OpenCV Mat
  cv::Mat raw_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

  // for (size_t i = 0; i < cam_bboxes_.size(); i++) {
  //   if (cam_bboxes_[i].x > 0 && (cam_bboxes_[i].x + cam_bboxes_[i].width) < raw_img.cols
  //       && cam_bboxes_[i].y > 0 && (cam_bboxes_[i].y + cam_bboxes_[i].height) < raw_img.rows
  //       && cam_bboxes_[i].width > 0 && cam_bboxes_[i].height > 0) {
  //     cv::Mat first_object_img = raw_img(cam_bboxes_[i]);
  //     imshow("First Object", first_object_img);
  //     break;
  //   }
  // }
  for (auto& bbox : cam_bboxes_) {
    cv::rectangle(raw_img, bbox, Scalar(0, 0, 255));

    for (auto& detect : detections){

    
      if (detect.Class != "car"){

      continue;
      }

      //ROS_INFO("%s \n", detect.Class.c_str());

    double width = (detect.xmax-detect.xmin);
    double height =(detect.ymax-detect.ymin);

    cv::Rect2d detect_car (detect.xmin, detect.ymin, width, height);

    cv::rectangle(raw_img, detect_car, Scalar(255, 0, 255));

    IoU(detect_car, bbox);

    //TO DO: Implement Intersection Over Union Funtion 

    //ROS_INFO("Image width: %f \n", width);

    //ROS_INFO("Image height: %f \n", height);

      }
  }

  cv::pyrDown(raw_img, raw_img, cv::Size(raw_img.cols/2, raw_img.rows/2));
  imshow("Output", raw_img);
  waitKey(1);
}

void CameraLidarFusion::recvLidarObjects(const avs_lecture_msgs::TrackedObjectArrayConstPtr& msg)
{
  // Do nothing until the coordinate transform from footprint to camera is valid,
  // because otherwise there is no point in detecting a lane!
  if (!looked_up_camera_transform_) {
    try {
      camera_transform_ = buffer_.lookupTransform("base_footprint", "camera", msg->header.stamp);
      looked_up_camera_transform_ = true; // Once the lookup is successful, there is no need to keep doing the lookup
                                          // because the transform is constant
    } catch (tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "%s", ex.what());
    }
    return;
  }

  // Create pinhole camera model instance and load
  // its parameters from the camera info
  // generated using the checkerboard calibration program
  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camera_info_);

  cam_bboxes_.clear();
  for (auto& obj : msg->objects) {
    tf2::Transform transform;
    tf2::convert(camera_transform_.transform, transform);
    cam_bboxes_.push_back(getCamBbox(obj, transform, model));
  }
}

// The purpose of this function is to convert the bounding box from the Lidar from a 3D frame to 2D frame that can be interpreted in the camera's reference frame
cv::Rect2d CameraLidarFusion::getCamBbox(const avs_lecture_msgs::TrackedObject& object, const tf2::Transform& transform, const image_geometry::PinholeCameraModel& model)
{

  // declare variables to store the maximum and minimum values of the 3D lidar bounding boxes
  std::vector<double> xvals(2);
  std::vector<double> yvals(2);
  std::vector<double> zvals(2);
  xvals[0] = -0.5 * object.bounding_box_scale.x;
  xvals[1] = 0.5 * object.bounding_box_scale.x;
  yvals[0] = -0.5 * object.bounding_box_scale.y;
  yvals[1] = 0.5 * object.bounding_box_scale.y;
  zvals[0] = -0.5 * object.bounding_box_scale.z;
  zvals[1] = 0.5 * object.bounding_box_scale.z;

// Loop through the x, y, and z, mins and maxes of the bounding boxes in order to and project these values in the camera's frame
  int min_x = 99999;
  int max_x = 0;
  int min_y = 99999;
  int max_y = 0;
  for (size_t i = 0; i < xvals.size(); i++) {
    for (size_t j = 0; j < xvals.size(); j++) {
      for (size_t k = 0; k < xvals.size(); k++) {
        tf2::Vector3 cam_vect = transform.inverse() * tf2::Vector3(object.pose.position.x + xvals[i],
                                                                   object.pose.position.y + yvals[i],
                                                                   object.pose.position.z + zvals[i]);
        cv::Point2d p = model.project3dToPixel(cv::Point3d(cam_vect.x(), cam_vect.y(), cam_vect.z()));
        if (p.x < min_x) {
          min_x = p.x;
        }
        if (p.y < min_y) {
          min_y = p.y;
        }
        if (p.x > max_x) {
          max_x = p.x;
        }
        if (p.y > max_y) {
          max_y = p.y;
        }
      }
    }
  }

  cv::Rect2d cam_bbox(min_x, min_y, max_x - min_x, max_y - min_y);
  return cam_bbox;
}

void CameraLidarFusion::recvCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg)
{
  camera_info_ = *msg;
}

void CameraLidarFusion::reconfig(CameraLidarFusionConfig& config, uint32_t level)
{
  tf2::Quaternion q;
  q.setRPY(config.roll, config.pitch, config.yaw);
  tf2::convert(q, camera_transform_.transform.rotation);
  tf2::convert(tf2::Vector3(config.x, config.y, config.z), camera_transform_.transform.translation);
  looked_up_camera_transform_ = true;

  camera_transform_.header.frame_id = "base_footprint";
  camera_transform_.child_frame_id = "camera_optical";
  broadcaster_.sendTransform(camera_transform_);
}

bool CameraLidarFusion::IoU(cv::Rect2d r1,cv::Rect2d r2)
{  

  //define maximum point on the rectangles, bottom right point of the rectangle


  double r1_xmax = r1.x + r1.width;
  double r1_ymax = r1.y + r1.height;

  double r2_xmax = r2.x + r2.width;
  double r2_ymax = r2.y + r2.height;


  // If one rectangle is on left side of other 
  if (r1.x >= r2_xmax || r2.x >= r1_xmax)
  {
   return false;
  }
    
         
   //If one rectangle is above other (Recall down is positive in OpenCV)
  if (r1.y >= r2_ymax || r2.y >= r1_ymax) 
  {
    return false;
  }


  //Area of rectangles
  double r1_area = r1.width * r1.height;
  double r2_area = r2.width * r2.height;

  //Locate top-left of intersected rectangle
  double ri_x = max(r1.x,r2.x);
  double ri_y = max(r1.y, r2.y);

  //Locate top-right of intersected rectangle
  double ri_xmax = min(r1_xmax,r2_xmax);
  double ri_ymax = min(r1_ymax,r2_ymax);

  //Calculate Intersected Width
  double ri_width = ri_xmax - ri_x;

  //Calculate Intersected Height
  double ri_height = ri_ymax - ri_y;

  //Area of intersection
  double ri_area = ri_height*ri_width;

  //Determine Intersection over Union
  double IoU = ri_area/((r1_area + r2_area)-ri_area);

  
  if(IoU > 0.65 )
  {
    ROS_INFO("IoU :%f", IoU);
    return true;
  }
  else
  {
  return false;
  }

  
}

}
