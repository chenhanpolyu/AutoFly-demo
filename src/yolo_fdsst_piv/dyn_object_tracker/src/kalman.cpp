/**
* Implementation of KalmanFilter class.
*
* @author: Hayk Martirosyan
* @date: 2014.11.15
*/

#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include "kalman.hpp"

KalmanFilter::KalmanFilter(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXd& Q,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& P,
    const int id)
  : A(A), C(C), Q(Q), R(R), P0(P),id(id),
    m(C.rows()), n(A.rows()), initialized(false),
    I(n, n)
    // , x_hat(n), x_hat_new(n)
{
  I.setIdentity();
}

KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::MatrixXd& x0, const Eigen::Vector3d& size_j) {
  x_hat = x0;
  P = P0;
  this->t0 = t0;
  t = t0;
  initialized = true;
  last_observe_t = t0;
  last_update_t = t0;
  size = size_j;
}
// void KalmanFilter::init() {
//   x_hat.setZero();
//   P = P0;
//   t0 = 0;
//   t = t0;
//   initialized = true;

// }
double KalmanFilter::duration(double current_t)
{
  return current_t - last_observe_t;
}
void KalmanFilter::update(const Eigen::MatrixXd& y) {

  if(!initialized)
    throw std::runtime_error("Filter is not initialized!");
  // std::cout<<"[KF] state: \n"<<x_hat<<std::endl;
  x_hat_new = A * x_hat;
  P = A*P*A.transpose() + Q;
  K = (P*C.transpose()).array()/(C*P*C.transpose() + R).array();
  x_hat_new += K * (y - C*x_hat_new);
  // P = (I - K*C)*P;
  P = (I - K*C)*P*(I - K*C).transpose() + K*R*K.transpose();
  x_hat = x_hat_new;
std::cout<<"[KF] state-1: \n"<<x_hat<<"\nobserve:\n"<<y<<"\nP:\n"<<P<<"\nK:\n"<<K<<std::endl;
  // t += dt;
}

// void KalmanFilter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd A) {

//   this->A = A;
//   this->dt = dt;
//   update(y);
// }

Eigen::MatrixXd KalmanFilter::forward_state(double current_time) 
{
 Eigen::Matrix2d A1;
 A1 <<1,current_time - last_update_t,0,1;
 return A1 * x_hat;
}
void KalmanFilter::update(double current_time, const Eigen::MatrixXd& R, Eigen::MatrixXd y, bool if_observed) {
  dt =  current_time - last_update_t;
  last_update_t = current_time;
  this->A<<1,dt,0,1;
  this->Q<<pow(dt,2)/4,pow(dt,3)/2,pow(dt,3)/2,pow(dt,2);
  this->Q *= 10;  //10:the variance of the acceleration of walking human
if (!if_observed)
y = this->A * x_hat;
else
  last_observe_t = current_time;
  // this->A = A;
  // this->Q = Q;
  this->R = R;
  update(y);

}