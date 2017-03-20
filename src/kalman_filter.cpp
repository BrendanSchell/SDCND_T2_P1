#include "kalman_filter.h"
#include "tools.h"

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
  this->x_ = this->F_ * this->x_;
  this->P_ = this->F_ * this->P_ * this->F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  MatrixXd z_pred = this->H_ * this->x_;
  MatrixXd y = z - z_pred ;
  MatrixXd S = this->H_ * this->P_ * this->H_.transpose() + this->R_;
  MatrixXd K = this->P_ * this->H_.transpose() * S.inverse();
  MatrixXd I = MatrixXd::Identity(4,4);
  this->x_ = this->x_ + K * y;
  this->P_ = (I-K*this->H_) * this->P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  Tools tools;
  MatrixXd Hj = tools.CalculateJacobian(this->x_);
  VectorXd hx = VectorXd(3);
  float px = this->x_[0];
  float py = this->x_[1];
  float vx = this->x_[2];
  float vy = this->x_[3];
  
  hx << sqrt(pow(px,2)+pow(py,2)), 
        atan2(py,px),
        (px*vx+py*vy)/sqrt(pow(px,2)+pow(py,2));
  MatrixXd y = z - hx;
  MatrixXd S = Hj* this->P_ * Hj.transpose() + this->R_;
  MatrixXd K = this->P_ * Hj.transpose() * S.inverse();
  MatrixXd I = MatrixXd::Identity(4,4);
  this->x_ = this->x_ + K * y;
  this->P_ = (I-K*Hj) * this->P_;
}
