#include "SyncedYoloData.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#define CALIBRATE_ALIGNMENT 0

//Display OpenCv Images for debugging 
#define DEBUG 0


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
    sub_lidar_.reset(new message_filters::Subscriber<avs_lecture_msgs::TrackedObjectArray>(n, "/lidar_ekf/object_tracks", 10));

    sync_yolo_data_.reset(new message_filters::Synchronizer<YoloSyncPolicy>(YoloSyncPolicy(10), *sub_img_, *sub_objects_));
    sync_yolo_data_->registerCallback(boost::bind(&SyncedYoloData::recvSyncedData, this, _1, _2));

    sync_lidar_data_.reset(new message_filters::Synchronizer<LidarSyncPolicy>(LidarSyncPolicy(10), *sub_img_, *sub_lidar_));
    sync_lidar_data_->registerCallback(boost::bind(&SyncedYoloData::recvLidarSynced, this, _1, _2));

    sub_lidar_objects_ = n.subscribe("/lidar_ekf/object_tracks", 5, &SyncedYoloData::recvLidarObjects, this);

    car_bboxes_ = n.advertise<avs_lecture_msgs::TrackedObjectArray>("fused_objects", 1);


    looked_up_camera_transform_ = false;

    previous_Box.clear();

    #if CALIBRATE_ALIGNMENT
    srv_.reset(new dynamic_reconfigure::Server<CameraLidarFusionConfig>);
    srv_->setCallback(boost::bind(&CameraLidarFusion::reconfig, this, _1, _2));
    #endif

    namedWindow("Sync_Output", WINDOW_NORMAL);
  }

  void SyncedYoloData::recvSyncedData(const sensor_msgs::ImageConstPtr& img_msg, const darknet_ros_msgs::BoundingBoxesConstPtr& object_msg)
  {
    if (!looked_up_camera_transform_){
      return;
    }

    
    Mat img_raw = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;


    One_run.clear();

    car_boxes.objects.clear();

    for (auto& bbox : myArr) 
    {

    // Access the 2D box of the LIDAR
    cv::Rect2d cam_box = bbox.first;


      for (auto& detect : object_msg->bounding_boxes)
        {

        //car_boxes.objects.clear();
        if (detect.Class != "car")
        {

        continue;
        }

        #if DEBUG
        ROS_INFO("%s \n", detect.Class.c_str());

        double width = (detect.xmax-detect.xmin);
        double height =(detect.ymax-detect.ymin);

        cv::Rect2d detect_car (detect.xmin, detect.ymin, width, height);

        cv::rectangle(img_raw, detect_car, Scalar(255, 0, 255));

        ROS_INFO("Box ID: %d", bbox_id );
        #endif


        int bbox_temp = bbox.second.id;

        
        //Compare 2D LIDAR box with YOLO bounding box, return true if IoU is over 50% [The third argument exist to store ID of the Latest successful IoU results, not used]
        if(IoU(cam_box, detect, bbox_temp))
        {

          #if DEBUG
          cv::rectangle(img_raw, cam_box, Scalar(0, 0, 255));
          #endif

          int bbox_id = bbox.second.id;      

          //ROS_INFO("Box ID: %d", bbox_id );


          previous_Box.push_back(bbox_id);


          
          avs_lecture_msgs::TrackedObject car_box = bbox.second;

          car_box.header = car_boxes.header;

        
          car_boxes.objects.push_back(car_box);
      
         }
      }
   }

    //car_bboxes_.publish(car_boxes);

    #if DEBUG

    cv::pyrDown(img_raw, img_raw, cv::Size(img_raw.cols/2, img_raw.rows/2));
    imshow("Sync_Output", img_raw);
    waitKey(1);

    #endif
    

  }

 
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


  // Create pinhole camera model instance and load
  // its parameters from the camera info
  // generated using the checkerboard calibration program
  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camera_info_);

  myArr.clear();

  for (auto& obj : object_msg->objects) {

    
  car_boxes.header = object_msg->header;
      
    tf2::Transform transform;
    tf2::convert(camera_transform_.transform, transform);

    // This array associates the 2D LIDAR box with the 3D LIDAR Box 
    myArr.push_back({getCamBbox(obj, transform, model), obj});

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
        
      
      }
    }
  }

  
  cv::Rect2d cam_bbox(min_x, min_y, max_x - min_x, max_y - min_y);

  return cam_bbox;
}

void SyncedYoloData::recvCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg)
{
  camera_info_ = *msg;
}

bool SyncedYoloData::IoU(cv::Rect2d r1, const darknet_ros_msgs::BoundingBox& detect, int fresh_objects)
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

    return true;

    
    One_run.push_back(fresh_objects);
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
  

  avs_lecture_msgs::TrackedObjectArray final_boxes;


  for (auto& IoU_output : previous_Box)
  {
   for (auto& final_box : msg->objects)
   {

     
     if(final_box.id != IoU_output)
     {
       continue;
     }


     final_boxes.objects.push_back(final_box);



   }
  }
  final_boxes.header = car_boxes.header;

  car_bboxes_.publish(final_boxes);
  
  
}



}