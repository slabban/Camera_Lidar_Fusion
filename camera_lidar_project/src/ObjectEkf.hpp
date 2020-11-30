// Prevent multiple declarations caused by including
//     this header file from multiple locations
#pragma once

#include <ros/ros.h>
#include <avs_lecture_msgs/TrackedObject.h>
#include <eigen3/Eigen/Dense>

namespace homework4
{
  // Define variable type 'StateVector' to be a 4x1 Eigen matrix
  typedef Eigen::Vector4d StateVector;

  // Define variable type 'StateMatrix' to be a 4x4 Eigen matrix
  typedef Eigen::Matrix4d StateMatrix;

  class ObjectEkf {

    public:

      // Constructor arguments are the initial state values and starting time stamp
      ObjectEkf(double x_pos0, double x_vel0,
                double y_pos0, double y_vel0, int id,
                const ros::Time& t0, const std::string& frame_id);

      // Sets the process noise standard deviations
      void setQ(double q_pos, double q_vel);

      // Sets the measurement noise standard deviation
      void setR(double r_pos);

      // Has it been a long time since the last time the filter has been
      // updated with a measurement sample?
      bool isStale();

      // Look up amount of time since filter was created
      double getAge();

      // Return the ID number property
      int getId();

      // Update filter without a measurement by running just the prediction step
      void updateFilterPredict(const ros::Time& current_time);

      // Full update of the filter with a cluster measurement
      void updateFilterMeasurement(const avs_lecture_msgs::TrackedObject& meas);

      // Create and return an Odometry output from filter state
      avs_lecture_msgs::TrackedObject getEstimate();
    
    private:

      // Estimate state, covariance, and current time stamp
      StateVector X_;
      StateMatrix P_;
      ros::Time estimate_stamp_;

      // Time of when the Kalman filter was created
      ros::Time spawn_stamp_;

      // Time of when the Kalman filter was last updated with a measurement sample
      ros::Time measurement_stamp_;

      // Process noise covariance
      StateMatrix Q_;

      // Measurement noise covariance
      Eigen::Matrix2d R_;

      // Data copied from measurement and not filtered
      geometry_msgs::Vector3 scale_;
      double z_;
      std::string frame_id_;

      // ID integer to make it unique
      int id_;

      // Methods to predict states and propagate uncertainty 
      StateVector statePrediction(double dt, const StateVector& old_state);
      StateMatrix stateJacobian(double dt, const StateVector& state);
      StateMatrix covPrediction(const StateMatrix& A, const StateMatrix& Q, const StateMatrix& old_cov);
  };

}
