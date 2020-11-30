// Header file for the class
#include "Homework4.hpp"

// Namespace matches ROS package name
namespace homework4
{

  // Constructor with global and private node handle arguments
  Homework4::Homework4(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    sub_detected_objects_ = n.subscribe("fused_objects", 1, &Homework4::recvObjects, this);
    pub_object_tracks_ = n.advertise<avs_lecture_msgs::TrackedObjectArray>("homework4/object_tracks", 1);

    update_timer_ = n.createTimer(ros::Duration(0.02), &Homework4::updateTimerCallback, this);

    srv_.setCallback(boost::bind(&Homework4::reconfig, this, _1, _2));
  }

  void Homework4::updateTimerCallback(const ros::TimerEvent& event)
  {
    // Delete stale objects that have not been observed for a while
    std::vector<size_t> stale_objects;
    for (size_t i = 0; i < object_ekfs_.size(); i++) {
      object_ekfs_[i].updateFilterPredict(event.current_real);
      if (object_ekfs_[i].isStale()) {
        stale_objects.push_back(i);
      }
    }
    for (int i = (int)stale_objects.size() - 1; i >= 0; i--) {
      object_ekfs_.erase(object_ekfs_.begin() + stale_objects[i]);
    }

    // Generate detected object outputs
    avs_lecture_msgs::TrackedObjectArray object_track_msg;
    object_track_msg.header.stamp = event.current_real;
    object_track_msg.header.frame_id = "base_footprint";
    for (size_t i = 0; i < object_ekfs_.size(); i++) {
      if (object_ekfs_[i].getAge() < cfg_.min_age) {
        continue;
      }
      object_track_msg.objects.push_back(object_ekfs_[i].getEstimate());
    }
    pub_object_tracks_.publish(object_track_msg);
  }

  void Homework4::recvObjects(const avs_lecture_msgs::TrackedObjectArrayConstPtr& msg)
  {
    // Vector to hold the EKF indices that have already been matched to an incoming object measurement
    std::vector<int> matched_object_indices;

    // Vector to hold array indices of objects to create new EKF instances from
    std::vector<int> new_object_indices;

    // Loop through all incoming object measurements and associate them with an existing EKF, or create
    // a new EKF instance and initialize its state to the measured values
    for (size_t i = 0; i < msg->objects.size(); i++) {
      const avs_lecture_msgs::TrackedObject& object = msg->objects[i];
      if (fabs(object.pose.position.y) > cfg_.y_window) {
        continue;
      }
      if (object.bounding_box_scale.z < cfg_.min_size_z) {
        continue;
      }
      
      // Loop through each existing EKF instance and find the one closest to the current object measurement
      double min_dist2 = INFINITY;
      int associated_track_idx = -1;
      for (size_t j = 0; j < object_ekfs_.size(); j++) {
        // If the current EKF instance has already been associated with a measurement, skip it and try the next one
        if (std::find(matched_object_indices.begin(), matched_object_indices.end(), j) != matched_object_indices.end()) {
          continue;
        }

        // Compute the distance between the EKF estimate of the position and the position of the measurement
        geometry_msgs::Point est_pos = object_ekfs_[j].getEstimate().pose.position;
        tf2::Vector3 est_pos_vect(est_pos.x, est_pos.y, 0.0);
        tf2::Vector3 meas_pos_vect(object.pose.position.x, object.pose.position.y, 0.0);
        double d2 = (meas_pos_vect - est_pos_vect).length2();

        // If the distance is the smallest so far, mark this EKF instance as the association candidate
        if (d2 < min_dist2) {
          min_dist2 = d2;
          associated_track_idx = (int)j;
        }
      }

      if ((associated_track_idx < 0) || (min_dist2 > (cfg_.max_match_dist * cfg_.max_match_dist))) {
        // If no EKF instances exist yet, or the closest match is too far away, mark this
        // object to create a new EKF instance to track it
        new_object_indices.push_back(i);
      } else {
        // Object measurement successfully associated with an existing EKF instance...
        // Update that EKF and mark it as already associated so another object in
        //     the same measurement array doesn't also get associated to it
        object_ekfs_[associated_track_idx].updateFilterMeasurement(object);
        matched_object_indices.push_back(associated_track_idx);
      }
    }

    // After trying to associate all incoming object measurements to existing EKF instances,
    // create new EKF instances to track the inputs that weren't associated with existing ones
    for (auto new_object_idx : new_object_indices) {
      object_ekfs_.push_back(ObjectEkf(msg->objects[new_object_idx].pose.position.x, 0.0, 
                                       msg->objects[new_object_idx].pose.position.y, 0.0, getUniqueId(),
                                       msg->objects[new_object_idx].header.stamp, msg->header.frame_id));
      object_ekfs_.back().setQ(cfg_.q_pos, cfg_.q_vel);
      object_ekfs_.back().setR(cfg_.r_pos);
    }
  }

  void Homework4::reconfig(Homework4Config& config, uint32_t level)
  {
    cfg_ = config;

    // Update Q and R matrices in each EKF instance
    for (size_t i = 0; i < object_ekfs_.size(); i++) {
      object_ekfs_[i].setQ(cfg_.q_pos, cfg_.q_vel);
      object_ekfs_[i].setR(cfg_.r_pos);
    }
  }

  int Homework4::getUniqueId()
  {
    int id = 0;
    bool done = false;
    while (!done) {
      done = true;
      for (auto& track : object_ekfs_) {
        if (track.getId() == id) {
          done = false;
          id++;
          break;
        }
      }
    }
    return id;
  }

}
