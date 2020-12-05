// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

// ROS headers
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Message headers
#include <avs_lecture_msgs/TrackedObjectArray.h>
#include <geometry_msgs/TwistStamped.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <camera_lidar_project/Homework4Config.h>

#include "ObjectEkf.hpp"

// Namespace matches ROS package name
namespace homework4 {

  class Homework4 {
    public:
      Homework4(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void updateTimerCallback(const ros::TimerEvent& event);
      void reconfig(Homework4Config& config, uint32_t level);
      void recvObjects(const avs_lecture_msgs::TrackedObjectArrayConstPtr& msg);
      int getUniqueId();

      // Methods to predict states and propagate uncertainty 
      StateVector statePrediction(double dt, const StateVector& old_state);
      StateMatrix stateJacobian(double dt, const StateVector& state);
      StateMatrix covPrediction(const StateMatrix& A, const StateMatrix& Q, const StateMatrix& old_cov);

      ros::Subscriber sub_detected_objects_;
      ros::Publisher pub_object_tracks_;
      ros::Timer marker_timer_;
      ros::Timer update_timer_;

      dynamic_reconfigure::Server<Homework4Config> srv_;
      Homework4Config cfg_;

      std::vector<ObjectEkf> object_ekfs_;
      static constexpr double DT = 1.0;

      int id = 0;
  };

}
