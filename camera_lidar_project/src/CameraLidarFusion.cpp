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
  sub_image_ = cam_nh.subscribe("/darknet_ros/detection_image", 5, &CameraLidarFusion::recvImage, this);
  sub_lidar_objects_ = n.subscribe("object_tracks", 5, &CameraLidarFusion::recvLidarObjects, this);
  sub_detections_ = n.subscribe("/darknet_ros/bounding_boxes", 5, &CameraLidarFusion::recvDetectionImage, this);
  //car_bboxes = n.advertise<FusedObjectArray>("fused_objects", 1);
  car_bboxes_ = n.advertise<avs_lecture_msgs::TrackedObjectArray>("fused_objects", 1);
  //label_object = n.advertise<visualization_msgs::MarkerArray>("labels", 1);
  bounding_boxes_ = n.advertise<darknet_ros_msgs::BoundingBoxes>("Car_Bounds", 1);
  detectionImagePublisher_ = n.advertise<sensor_msgs::Image>("Car_Detections", 1);

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

  //car_boxes.header = msg->header;

    for (auto& bbox : cam_bboxes_) {
    //cv::rectangle(raw_img, bbox, Scalar(0, 0, 255));

    for (auto& detect : detections){

      car_boxes.objects.clear();
      if (detect.Class != "car"){

      continue;
      }

      //ROS_INFO("%s \n", detect.Class.c_str());

    double width = (detect.xmax-detect.xmin);
    double height =(detect.ymax-detect.ymin);

    cv::Rect2d detect_car (detect.xmin, detect.ymin, width, height);

    //cv::rectangle(raw_img, detect_car, Scalar(255, 0, 255));

    //ROS_INFO("Box ID: %d", bbox_id );
    
    //Assign the label to the lidar box, Identified by the unique box id
    if(IoU(bbox, detect))
    {

      ROS_INFO("This Box is a car: %d", bbox_id );
      cv::rectangle(raw_img, bbox, Scalar(0, 0, 255));

      //car_boxes.objects.clear();

      car_boxid = bbox_id;

      avs_lecture_msgs::TrackedObject car_box;
      //FusedObject car_box;

      car_box.header = car_boxes.header;
      // Fill in the rest of the 'box' variable
      //         - Set `spawn_time` to the current ROS time
      //         - Increment the 'id' field for each cluster
      //         - Populate 'pose.position' with the midpoint between min_point and max_point
      //         - Populate 'pose.orientation' with an identity quaternion
      //         - Leave 'velocity.linear' and 'velocity.angular' unpopulated
      //         - Populate 'bounding_box_scale' with data from min_point and max_point
      //         - Leave 'bounding_box_offset' unpopulated
      //car_box.Class = detect.Class;
      car_box.id = car_boxid;
      //car_box.spawn_time = ros::WallTime::now();
      car_box.bounding_box_scale.x = bbox_scale_x;
      car_box.bounding_box_scale.y = bbox_scale_y;
      car_box.bounding_box_scale.z = bbox_scale_z;
      car_box.pose.position.x = bbox_pos_x;
      car_box.pose.position.y = bbox_pos_y;
      car_box.pose.position.z = bbox_pos_z;
      car_box.pose.orientation.w = 1.0;
     

      car_boxes.objects.push_back(car_box);

      car_bboxes_.publish(car_boxes);



    }

    //ROS_INFO("Image width: %f \n", width);

    //ROS_INFO("Image height: %f \n", height);

      }
    }
  
  
  
  cv::pyrDown(raw_img, raw_img, cv::Size(raw_img.cols/2, raw_img.rows/2));
  imshow("Output", raw_img);
  waitKey(1);

/*
  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = msg -> header.stamp;
  cvImage.header.frame_id = "detection_image";
  cvImage.encoding = sensor_msgs::image_encodings::BGR8;
  cvImage.image = raw_img;
  detectionImagePublisher_.publish(*cvImage.toImageMsg());
*/
}

// This function is called whenever a new image is received from either
// the live running camera driver, a bag file recording, or the simulated
// image coming from Gazebo
void CameraLidarFusion::recvImage(const sensor_msgs::ImageConstPtr& msg)
{

  //car_boxes.header = msg->header;

  // Convert ROS image message into an OpenCV Mat
   raw_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

  // for (size_t i = 0; i < cam_bboxes_.size(); i++) {
  //   if (cam_bboxes_[i].x > 0 && (cam_bboxes_[i].x + cam_bboxes_[i].width) < raw_img.cols
  //       && cam_bboxes_[i].y > 0 && (cam_bboxes_[i].y + cam_bboxes_[i].height) < raw_img.rows
  //       && cam_bboxes_[i].width > 0 && cam_bboxes_[i].height > 0) {
  //     cv::Mat first_object_img = raw_img(cam_bboxes_[i]);
  //     imshow("First Object", first_object_img);
  //     break;
  //   }
  // }


  // Implement message synchronizer 

  // implement wall time (ros::walltime for Bag files) to gauge processing time, compare agaist 10HZ

  // run rostopic hz to see publsih rate of topics

/*
  
  for (auto& bbox : cam_bboxes_) {
    //cv::rectangle(raw_img, bbox, Scalar(0, 0, 255));

    for (auto& detect : detections){

      car_boxes.objects.clear();
      if (detect.Class != "car"){

      continue;
      }

      //ROS_INFO("%s \n", detect.Class.c_str());

    double width = (detect.xmax-detect.xmin);
    double height =(detect.ymax-detect.ymin);

    cv::Rect2d detect_car (detect.xmin, detect.ymin, width, height);

    //cv::rectangle(raw_img, detect_car, Scalar(255, 0, 255));

    //ROS_INFO("Box ID: %d", bbox_id );
    
    //Assign the label to the lidar box, Identified by the unique box id
    if(IoU(bbox, detect))
    {

      ROS_INFO("This Box is a car: %d", bbox_id );
      cv::rectangle(raw_img, bbox, Scalar(0, 0, 255));

      //car_boxes.objects.clear();

      car_boxid = bbox_id;

      avs_lecture_msgs::TrackedObject car_box;
      //FusedObject car_box;

      car_box.header = car_boxes.header;
      // Fill in the rest of the 'box' variable
      //         - Set `spawn_time` to the current ROS time
      //         - Increment the 'id' field for each cluster
      //         - Populate 'pose.position' with the midpoint between min_point and max_point
      //         - Populate 'pose.orientation' with an identity quaternion
      //         - Leave 'velocity.linear' and 'velocity.angular' unpopulated
      //         - Populate 'bounding_box_scale' with data from min_point and max_point
      //         - Leave 'bounding_box_offset' unpopulated
      //car_box.Class = detect.Class;
      car_box.id = car_boxid;
      //car_box.spawn_time = ros::WallTime::now();
      car_box.bounding_box_scale.x = bbox_scale_x;
      car_box.bounding_box_scale.y = bbox_scale_y;
      car_box.bounding_box_scale.z = bbox_scale_z;
      car_box.pose.position.x = bbox_pos_x;
      car_box.pose.position.y = bbox_pos_y;
      car_box.pose.position.z = bbox_pos_z;
      car_box.pose.orientation.w = 1.0;
     

      car_boxes.objects.push_back(car_box);

      car_bboxes_.publish(car_boxes);



    }

    //ROS_INFO("Image width: %f \n", width);

    //ROS_INFO("Image height: %f \n", height);

      }
      
  }
  
  
  cv::pyrDown(raw_img, raw_img, cv::Size(raw_img.cols/2, raw_img.rows/2));
  imshow("Output", raw_img);
  waitKey(1);

  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = msg -> header.stamp;
  cvImage.header.frame_id = "detection_image";
  cvImage.encoding = sensor_msgs::image_encodings::BGR8;
  cvImage.image = raw_img;
  detectionImagePublisher_.publish(*cvImage.toImageMsg());
    */
}

void CameraLidarFusion::recvLidarObjects(const avs_lecture_msgs::TrackedObjectArrayConstPtr& msg)
{

  //revisit
  //fused_objects = msg ->objects;


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

  //car_boxes.header = msg->header;

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

  //car_boxes.header = msg->header;
/*
  //fused_objects = msg->objects;
    for (auto& bbox : cam_bboxes_) {
    //cv::rectangle(raw_img, bbox, Scalar(0, 0, 255));

    for (auto& detect : detections){

      car_boxes.objects.clear();
      if (detect.Class != "car"){

      continue;
      }

      //ROS_INFO("%s \n", detect.Class.c_str());

    double width = (detect.xmax-detect.xmin);
    double height =(detect.ymax-detect.ymin);

    cv::Rect2d detect_car (detect.xmin, detect.ymin, width, height);

    //cv::rectangle(raw_img, detect_car, Scalar(255, 0, 255));

    //ROS_INFO("Box ID: %d", bbox_id );
    
    //Assign the label to the lidar box, Identified by the unique box id
    if(IoU(detect_car, bbox))
    {

      ROS_INFO("This Box is a car: %d", bbox_id );
      //cv::rectangle(raw_img, bbox, Scalar(0, 0, 255));

      //car_boxes.objects.clear();

      car_boxid = bbox_id;

      avs_lecture_msgs::TrackedObject car_box;
      //FusedObject car_box;

      car_box.header = car_boxes.header;
      // Fill in the rest of the 'box' variable
      //         - Set `spawn_time` to the current ROS time
      //         - Increment the 'id' field for each cluster
      //         - Populate 'pose.position' with the midpoint between min_point and max_point
      //         - Populate 'pose.orientation' with an identity quaternion
      //         - Leave 'velocity.linear' and 'velocity.angular' unpopulated
      //         - Populate 'bounding_box_scale' with data from min_point and max_point
      //         - Leave 'bounding_box_offset' unpopulated
      //car_box.Class = detect.Class;
      car_box.id = car_boxid;
      //car_box.spawn_time = ros::WallTime::now();
      car_box.bounding_box_scale.x = bbox_scale_x;
      car_box.bounding_box_scale.y = bbox_scale_y;
      car_box.bounding_box_scale.z = bbox_scale_z;
      car_box.pose.position.x = bbox_pos_x;
      car_box.pose.position.y = bbox_pos_y;
      car_box.pose.position.z = bbox_pos_z;
      car_box.pose.orientation.w = 1.0;
     

      car_boxes.objects.push_back(car_box);

      car_bboxes.publish(car_boxes);



    }
  }
 }
 */



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

//ROS_INFO("Box ID: %d", object.id );

// Loop through the x, y, and z of the bounding boxes in order to and project these values in the camera's frame
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

        //Bound the maximum and minimum values of the rectangle to fit in the cameras pixel coordinates
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
        
       // car_boxes.header = object.header;
        // bbox_id = object.id;
        // bbox_scale_x = object.bounding_box_scale.x;
        // bbox_scale_y = object.bounding_box_scale.y;
        // bbox_scale_z = object.bounding_box_scale.z;
        // bbox_pos_x = object.pose.position.x;
        // bbox_pos_y = object.pose.position.y;
        // bbox_pos_z = object.pose.position.z;
        // bbox_orientation= object.pose.orientation.w;



        //ROS_INFO("Box ID: %d", bbox_id );

      }
    }
  }
  car_boxes.header = object.header;
  bbox_id = object.id;
  bbox_scale_x = object.bounding_box_scale.x;
  bbox_scale_y = object.bounding_box_scale.y;
  bbox_scale_z = object.bounding_box_scale.z;
  bbox_pos_x = object.pose.position.x;
  bbox_pos_y = object.pose.position.y;
  bbox_pos_z = object.pose.position.z;
  bbox_orientation= object.pose.orientation.w;

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

bool CameraLidarFusion::IoU(cv::Rect2d r1, darknet_ros_msgs::BoundingBox& detect)
{  

 //cv::Rect2d r2
  //define maximum point on the rectangles, bottom right point of the rectangle
 
  double r1_xmax = r1.br().x;
  double r1_ymax = r1.br().y;

  //double r2_xmax = r2.br().x;
  //double r2_ymax = r2.br().y;

  double r2_xmin = detect.xmin;
  double r2_ymin = detect.ymin;

  double r2_xmax = detect.xmax;
  double r2_ymax = detect.ymax;

  double width = (detect.xmax-detect.xmin);
  double height =(detect.ymax-detect.ymin);


  // If one rectangle is on left side of other 
  if (r1.x >= r2_xmax || detect.xmin >= r1_xmax)
  {
   return false;
  }
    
         
   //If one rectangle is above other (Recall down is positive in OpenCV)
  if (r1.y >= r2_ymax || detect.ymin >= r1_ymax) 
  {
    return false;
  }


  //Area of rectangles
  double r1_area = r1.area();
  //double r2_area = r2.area();
  double r2_area = width * height;


  //Locate top-left of intersected rectangle
  //double ri_x = max(r1.x,r2.x);
  //double ri_y = max(r1.y, r2.y);

  double ri_x = max(r1.x, r2_xmin);
  double ri_y = max(r1.y, r2_ymin);

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

/*
//revisit
void CameraLidarFusion::generateBoundingBoxes(const avs_lecture_msgs::TrackedObject& object)
{
  //fused_objects = msg->objects;

  for (size_t i = 0; i < fused_objects.size(); i++)
  {
    
  }
  

 

    //for(auto& fused_object : fused_objects)
    //{

      //ROS_INFO("ID: %d", fused_object.id);

      if (bbox_id = car_box)
      {

        ROS_INFO("oink");
        pub_bboxes_.publish(fused_objects);
      }
      

    //}
}
*/

}

