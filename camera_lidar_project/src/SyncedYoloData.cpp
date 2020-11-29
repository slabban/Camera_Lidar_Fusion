#include "SyncedYoloData.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

//changed
namespace camera_lidar_project
{

  SyncedYoloData::SyncedYoloData(ros::NodeHandle& n, ros::NodeHandle& pn) :
  listener_(buffer_)

  {

    ros::NodeHandle cam_nh("camera");
    sub_cam_info_ = cam_nh.subscribe("camera_info", 1, &SyncedYoloData::recvCameraInfo, this);
    sub_img_.reset(new message_filters::Subscriber<sensor_msgs::Image>(n, "/darknet_ros/detection_image", 5));
    sub_objects_.reset(new message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>(n, "/darknet_ros/bounding_boxes", 5));
    sub_lidar_.reset(new message_filters::Subscriber<avs_lecture_msgs::TrackedObjectArray>(n, "detected_objects", 5));

    sync_yolo_data_.reset(new message_filters::Synchronizer<YoloSyncPolicy>(YoloSyncPolicy(10), *sub_img_, *sub_objects_));
    sync_yolo_data_->registerCallback(boost::bind(&SyncedYoloData::recvSyncedData, this, _1, _2));

    sync_lidar_data_.reset(new message_filters::Synchronizer<LidarSyncPolicy>(LidarSyncPolicy(10), *sub_img_, *sub_lidar_));
    sync_lidar_data_->registerCallback(boost::bind(&SyncedYoloData::recvLidarSynced, this, _1, _2));

    sub_lidar_objects_ = n.subscribe("object_tracks", 5, &SyncedYoloData::recvLidarObjects, this);

    car_bboxes_ = n.advertise<avs_lecture_msgs::TrackedObjectArray>("fused_objects", 1);

    looked_up_camera_transform_ = false;

    #if CALIBRATE_ALIGNMENT
    srv_.reset(new dynamic_reconfigure::Server<CameraLidarFusionConfig>);
    srv_->setCallback(boost::bind(&CameraLidarFusion::reconfig, this, _1, _2));
    #endif

    namedWindow("Sync_Output", WINDOW_NORMAL);
  }

  void SyncedYoloData::recvSyncedData(const sensor_msgs::ImageConstPtr& img_msg, const darknet_ros_msgs::BoundingBoxesConstPtr& object_msg)
  {
    
    Mat img_raw = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;

    //car_boxes.objects.clear();

    detections = object_msg->bounding_boxes;

    car_boxes.objects.clear();

    for (auto& bbox : myArr) {
    //cv::rectangle(raw_img, bbox, Scalar(0, 0, 255));
    //vec.at(0);
    //cv::Rect2d& lebox = boost::get<cv::Rect2d>(vec[0]);

    cv::Rect2d cam_box = bbox.first;

    //int le_id = bbox.second;

    //car_boxes.objects.clear();

    for (auto& detect : object_msg->bounding_boxes){

      //car_boxes.objects.clear();
      if (detect.Class != "car"){

      continue;
      }

      //ROS_INFO("%s \n", detect.Class.c_str());

    double width = (detect.xmax-detect.xmin);
    double height =(detect.ymax-detect.ymin);

    //cv::Rect2d detect_car (detect.xmin, detect.ymin, width, height);

    //cv::rectangle(img_raw, detect_car, Scalar(255, 0, 255));

    //ROS_INFO("Box ID: %d", bbox_id );
    
    //Assign the label to the lidar box, Identified by the unique box id
    if(IoU(cam_box, detect))
    {

      //ROS_INFO("This Box is a car: %d", le_id );
      cv::rectangle(img_raw, cam_box, Scalar(0, 0, 255));

      /*
      for (size_t i = 0; i < car_boxes.objects.size(); i++)
      {
         code 
      }
      */

      //car_boxes.objects.clear();

      //car_boxid = bbox_id;

      
      avs_lecture_msgs::TrackedObject car_box = bbox.second;
      //FusedObject car_box;

      car_box.header = car_boxes.header;;
      // Fill in the rest of the 'box' variable
      //         - Set `spawn_time` to the current ROS time
      //         - Increment the 'id' field for each cluster
      //         - Populate 'pose.position' with the midpoint between min_point and max_point
      //         - Populate 'pose.orientation' with an identity quaternion
      //         - Leave 'velocity.linear' and 'velocity.angular' unpopulated
      //         - Populate 'bounding_box_scale' with data from min_point and max_point
      //         - Leave 'bounding_box_offset' unpopulated
      //car_box.Class = detect.Class;
      //car_box.id = car_boxid;
      //car_box.spawn_time = ros::WallTime::now();
      // car_box.bounding_box_scale.x = bbox_scale_x;
      // car_box.bounding_box_scale.y = bbox_scale_y;
      // car_box.bounding_box_scale.z = bbox_scale_z;
      // car_box.pose.position.x = bbox_pos_x;
      // car_box.pose.position.y = bbox_pos_y;
      // car_box.pose.position.z = bbox_pos_z;
      // car_box.pose.orientation.w = 1.0;
    

      car_boxes.objects.push_back(car_box);
      
      
      car_bboxes_.publish(car_boxes);
      


    }

      }
    }
    /*
    for (auto& bbox : object_msg->bounding_boxes) {

      double width = (bbox.xmax-bbox.xmin);
      double height =(bbox.ymax-bbox.ymin);

      cv::Point2d corner(bbox.xmin, bbox.ymin);
      rectangle(img_raw, cv::Rect(bbox.xmin, bbox.ymin, width, height), Scalar(0, 255, 0));
      //putText(raw_img, bbox.label, corner, FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 255, 255));
    }

    for (auto& bbox : cam_bboxes_) {
    cv::rectangle(img_raw, bbox, Scalar(0, 0, 255));
    }
    */

    cv::pyrDown(img_raw, img_raw, cv::Size(img_raw.cols/2, img_raw.rows/2));
    imshow("Sync_Output", img_raw);
    waitKey(1);
    
  // create a global message and an apporximate sycronizer for the Lidar and Camera info, implement the IOU algorithm there!

  }

 //void SyncedYoloData::recvLidarSynced(const darknet_ros_msgs::BoundingBoxesConstPtr& bbox_msg, const avs_lecture_msgs::TrackedObjectArrayConstPtr& object_msg)
  void SyncedYoloData::recvLidarSynced(const sensor_msgs::ImageConstPtr& img_msg, const avs_lecture_msgs::TrackedObjectArrayConstPtr& object_msg)
  {

  
  if (!looked_up_camera_transform_) {
    try {
      camera_transform_ = buffer_.lookupTransform("base_footprint", "camera", img_msg->header.stamp);
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

  //car_boxes.objects.clear();
  //car_boxes.header = object_msg->header;
  //car_boxes.objects = object_msg->objects;
  //car_bboxes_.publish(car_boxes);

  //cam_bboxes_.clear();

  //vec.clear();

  myArr.clear();

  for (auto& obj : object_msg->objects) {

    
  car_boxes.header = object_msg->header;
    /*
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
    car_box.id = obj.id;
    car_box.spawn_time = ros::Time::now();
    car_box.bounding_box_scale.x = obj.bounding_box_scale.x;
    car_box.bounding_box_scale.y = obj.bounding_box_scale.y;
    car_box.bounding_box_scale.z = obj.bounding_box_scale.z;
    car_box.pose.position.x = obj.pose.position.x;
    car_box.pose.position.y = obj.pose.position.y;
    car_box.pose.position.z = obj.pose.position.z;
    car_box.pose.orientation.w = 1.0;


    car_boxes.objects.push_back(car_box);

    //car_bboxes_.publish(car_boxes);
    */
    //int box_id = obj.id;
      
    tf2::Transform transform;
    tf2::convert(camera_transform_.transform, transform);

    myArr.push_back({getCamBbox(obj, transform, model), obj});

    //vec.at
  }

  
  
}

cv::Rect2d SyncedYoloData::getCamBbox(const avs_lecture_msgs::TrackedObject& object, const tf2::Transform& transform, const image_geometry::PinholeCameraModel& model)
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
        
      


        //ROS_INFO("Box ID: %d", bbox_id );
      
      }
    }
  }
  /*
  avs_lecture_msgs::TrackedObject car_box;
  //FusedObject car_box;

  car_box.header = object.header;
  // Fill in the rest of the 'box' variable
  //         - Set `spawn_time` to the current ROS time
  //         - Increment the 'id' field for each cluster
  //         - Populate 'pose.position' with the midpoint between min_point and max_point
  //         - Populate 'pose.orientation' with an identity quaternion
  //         - Leave 'velocity.linear' and 'velocity.angular' unpopulated
  //         - Populate 'bounding_box_scale' with data from min_point and max_point
  //         - Leave 'bounding_box_offset' unpopulated
  //car_box.Class = detect.Class;
  car_box.id = object.id;
  car_box.spawn_time = ros::Time::now();
  car_box.bounding_box_scale.x = object.bounding_box_scale.x;
  car_box.bounding_box_scale.y = object.bounding_box_scale.y;
  car_box.bounding_box_scale.z = object.bounding_box_scale.z;
  car_box.pose.position.x = object.pose.position.x;
  car_box.pose.position.y = object.pose.position.y;
  car_box.pose.position.z = object.pose.position.z;
  car_box.pose.orientation.w = 1.0;

  car_boxes.objects.push_back(car_box);

  car_bboxes_.publish(car_boxes);
  */
  
  cv::Rect2d cam_bbox(min_x, min_y, max_x - min_x, max_y - min_y);

  return cam_bbox;
}

void SyncedYoloData::recvCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg)
{
  camera_info_ = *msg;
}

bool SyncedYoloData::IoU(cv::Rect2d r1, const darknet_ros_msgs::BoundingBox& detect)
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

  
  if(IoU > 0.5 )
  {
    ROS_INFO("IoU :%f", IoU);
    return true;
  }
  else
  {
  return false;
  }

  
}

void SyncedYoloData::reconfig(CameraLidarFusionConfig& config, uint32_t level)
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

void SyncedYoloData::recvLidarObjects(const avs_lecture_msgs::TrackedObjectArrayConstPtr& msg)
{
  //car_bboxes_.publish(car_boxes);
}



}