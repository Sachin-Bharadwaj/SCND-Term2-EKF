#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd error = z - H_ * x_;
  MatrixXd Ht  = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  // new estimate
  x_ = x_ + (K * error);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = sqrt(px * px + py * py);
  float phi = 0.0; // for the case when px is 0
  if(fabs(px)>0.0001){
    phi = atan2(py,px); // [-pi,pi]
  }
  float rhodot = 0.0;
  if(fabs(rho)>0.0001){
    rhodot = (px * vx + py * vy)/rho;
   }

  float rho_res = z(0) - rho;
  float phi_res = z(1) - phi;
  // check if phi is in [-pi,pi]
  if(phi_res >= PI)
  {
    cout << "Inside gr than PI" << endl;
    phi_res = phi_res - 2*PI;
  }
  else if(phi_res < -PI){
   cout << "Inside less than -PI" << endl;
    phi_res = phi_res + 2*PI; 
  }
  float rhodot_res = z(2) - rhodot;
  VectorXd res = VectorXd(3);
  res << rho_res, phi_res, rhodot_res;
  
  MatrixXd Ht  = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  // new estimate
  x_ = x_ + (K * res);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;  
}
