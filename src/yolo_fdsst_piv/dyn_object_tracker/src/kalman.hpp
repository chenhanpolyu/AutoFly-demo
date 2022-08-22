/**
* Kalman filter implementation using Eigen. Based on the following
* introductory paper:
*
*     http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <Eigen/Dense>

#pragma once

class KalmanFilter {

public:

  /**
  * Create a Kalman filter with the specified matrices.
  *   A - System dynamics matrix
  *   C - Output matrix
  *   Q - Process noise covariance
  *   R - Measurement noise covariance
  *   P - Estimate error covariance
  */
  KalmanFilter(
      const Eigen::MatrixXd& A,
      const Eigen::MatrixXd& C,
      const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R,
      const Eigen::MatrixXd& P,
      const int id
  );

  /**
  * Create a blank estimator.
  */
  KalmanFilter();

  /**
  * Initialize the filter with initial states as zero.
  */
  void init();

  /**
  * Initialize the filter with a guess for initial states.
  */
  void init(double t0, const Eigen::MatrixXd& x0, const Eigen::Vector3d &size_j);

  /**
  * Update the estimated state based on measured values. The
  * time step is assumed to remain constant.
  */
  void update(const Eigen::MatrixXd& y);

  /**
  * Update the estimated state based on measured values,
  * using the given time step and dynamics matrix.
  */
  void update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A);
  void update(double current_time, const Eigen::MatrixXd& R, Eigen::MatrixXd y, bool if_observed=true);
  /**
  * Return the current state and time.
  */
  Eigen::MatrixXd state() { return x_hat; };
  Eigen::MatrixXd forward_state(double current_time);
  double time() { return t; };
  double duration(double current_t);
  int id;
  double last_observe_t, last_update_t;
  Eigen::Vector3d size;
  Eigen::MatrixXd P;
private:

  // Matrices for computation
  Eigen::MatrixXd A, C, Q, R, K, P0;

  // System dimensions
  int m, n;

  // Initial and current time
  double t0, t;

  // Discrete time step
  double dt;

 //
 
  // Is the filter initialized?
  bool initialized;

  // n-size identity
  Eigen::MatrixXd I;

  // Estimated states
  Eigen::MatrixXd x_hat, x_hat_new;
};
