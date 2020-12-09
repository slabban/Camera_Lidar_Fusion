#include "ObjectEkf.hpp"

namespace lidar_ekf
{

  ObjectEkf::ObjectEkf(double x_pos0, double x_vel0,
                       double y_pos0, double y_vel0, int id,
                       const ros::Time& t0, const std::string& frame_id)
  {
    // Initialize estimate covariance to identity
    P_.setIdentity();

    // Initialize state estimate to input arguments
    X_ << x_pos0, x_vel0, y_pos0, y_vel0;

    // Initialize all time stamps to the starting time
    estimate_stamp_ = t0;
    measurement_stamp_ = t0;
    spawn_stamp_ = t0;

    // Set dummy values for Q and R. These should be set from the code
    // instantiating the ObjectEkf class using setQ() and setR()
    //setQ(1.0, 1.0);
    //setR(1.0);

    frame_id_ = frame_id;
    id_ = id;
  }

  void ObjectEkf::updateFilterPredict(const ros::Time& current_time)
  {
    // Calculate time difference between current time and filter state
    double dt = (current_time - estimate_stamp_).toSec();
    if (fabs(dt) > 2) {
      // Large time jump detected... just reset to the current time
      spawn_stamp_ = current_time;
      estimate_stamp_ = current_time;
      measurement_stamp_ = current_time;
      return;
    }

    // Propagate estimate prediction and update estimate with result
    StateMatrix A = stateJacobian(dt, X_);
    X_ = statePrediction(dt, X_);
    P_ = covPrediction(A, Q_, P_);
    estimate_stamp_ = current_time;
  }

  void ObjectEkf::updateFilterMeasurement(const avs_lecture_msgs::TrackedObject& meas)
  {
    // Calculate time difference between measurement and filter state
    double dt = (meas.header.stamp - estimate_stamp_).toSec();
    if (fabs(dt) > 2) {
      // Large time jump detected... reset filter to this measurement
      X_ << meas.pose.position.x, 0.0, meas.pose.position.y, 0.0;
      P_.setIdentity();
      spawn_stamp_ = meas.header.stamp;
      estimate_stamp_ = meas.header.stamp;
      measurement_stamp_ = meas.header.stamp;
      return;
    }

    // Prediction step
    StateMatrix A = stateJacobian(dt, X_);
    StateVector predicted_state = statePrediction(dt, X_);
    StateMatrix predicted_cov = covPrediction(A, Q_, P_);

    // Measurement update
    // Define measurement matrix 
    Eigen::Matrix<double, 2, 4> C;
    C.row(0) << 1, 0, 0, 0;
    C.row(1) << 0, 0, 1, 0;

    Eigen::Vector2d meas_vect;
    meas_vect << meas.pose.position.x, meas.pose.position.y;

    // Compute expected measurement based on predicted_state 
    Eigen::Vector2d expected_meas;
    expected_meas << predicted_state(0), predicted_state(2);

    // Compute residual covariance matrix 
    Eigen::Matrix2d S;
    S = C * predicted_cov * C.transpose() + R_;

    // Compute Kalman gain #done
    Eigen::Matrix<double, 4, 2> K;
    K = predicted_cov * C.transpose() * S.inverse();

    // Update state estimate #done
    //X_ << meas.pose.position.x, 0.0, meas.pose.position.y, 0.0;
    X_ = predicted_state + K * (meas_vect - expected_meas);

    // Update estimate covariance #done
    P_ = (StateMatrix::Identity() - K * C) * predicted_cov;

    // Set estimate time stamp and latest measurement time stamp to the stamp in the input argument
    estimate_stamp_ = meas.header.stamp;
    measurement_stamp_ = meas.header.stamp;

    // Copy information that is not filtered
    scale_ = meas.bounding_box_scale;
    z_ = meas.pose.position.z;
  }

  StateVector ObjectEkf::statePrediction(double dt, const StateVector& old_state) {
    // Propagate the old_state argument through the discrete state equations and put the results in new_state #Done
    //       The 'dt' argument of this method is the sample time to use
  
   double x_speed_est = old_state(1);
   double y_speed_est = old_state(3);

   // Implement state prediction step
   StateVector new_state;
   new_state(0) = old_state(0) + dt * x_speed_est;
   new_state(1) = old_state(1); 
   new_state(2) = old_state(2) + dt * y_speed_est;
   new_state(3) = old_state(3);

   return new_state;
  }

  StateMatrix ObjectEkf::stateJacobian(double dt, const StateVector& state) {
    // Fill in the elements of the state Jacobian 
    // The 'dt' argument of this method is the sample time to use
    StateMatrix A;
    A.row(0) << 1, dt, 0, 0;
    A.row(1) << 0, 1, 0, 0;
    A.row(2) << 0, 0, 1, dt;
    A.row(3) << 0, 0, 0, 1;
    return A;
  }

  StateMatrix ObjectEkf::covPrediction(const StateMatrix& A, const StateMatrix& Q, const StateMatrix& old_cov) {
    // Propagate the old_cov argument through the covariance prediction equation and put the result in new_cov 
    StateMatrix new_cov;
    new_cov = A * old_cov * A.transpose() + Q;
    return new_cov;
  }

  // Has it been a long time since the last time the filter has been
  // updated with a measurement sample?
  bool ObjectEkf::isStale() {
    return (estimate_stamp_ - measurement_stamp_) > ros::Duration(0.5);
  }

  // Look up amount of time since filter was created
  double ObjectEkf::getAge() {
    return (estimate_stamp_ - spawn_stamp_).toSec();
  }

  // Sets the process noise standard deviations
  void ObjectEkf::setQ(double q_pos, double q_vel)
  {
    // Populate Q_ with q_pos and q_vel
    Q_.setZero();
    Q_(0, 0) = q_pos * q_pos;
    Q_(1, 1) = q_vel * q_vel;
    Q_(2, 2) = q_pos * q_pos;
    Q_(3, 3) = q_vel * q_vel;

  }

  // Sets the measurement noise standard deviation
  void ObjectEkf::setR(double r_pos)
  {
    // Populate R_ with r_pos
    R_.setZero();
    R_(0, 0) = r_pos * r_pos;
    R_(1, 1) = r_pos * r_pos;

  }

  // Return the ID number property
  int ObjectEkf::getId()
  {
    return id_;
  }

  // Create and return a DetectedObject output from filter state
  avs_lecture_msgs::TrackedObject ObjectEkf::getEstimate() {
    avs_lecture_msgs::TrackedObject estimate_output;
    estimate_output.header.stamp = estimate_stamp_;
    estimate_output.header.frame_id = frame_id_;
    estimate_output.id = id_;
    estimate_output.pose.position.z = z_;
    estimate_output.pose.orientation.w = 1.0;
    estimate_output.bounding_box_scale = scale_;

    // Populate output x and y position with filter estimate
    estimate_output.pose.position.x = X_(0);
    estimate_output.pose.position.y = X_(2);

    // Populate output x and y velocity with filter estimate
    estimate_output.velocity.linear.x = X_(1);
    estimate_output.velocity.linear.y = X_(3);

    return estimate_output;
  }

}