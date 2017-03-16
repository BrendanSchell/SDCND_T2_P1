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
  this.x_ = this.F_ * this.x_ + u;
  this.P_ = this.F_ * this.P_ * this.F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  MatrixXd y = z - this.H_*this.x_;
  MatrixXd S = this.H_ * this.P_ * this.H_.transpose() + this.R_;
  MatrixXd K = this.P_ * this.H_.transpose() * S.inverse();
  MatrixXd I = MatrixXd::Identity(4,4);
  this.x_ = this.x_ + K * y;
  this.P_ = (I-K*this.H_) * this.P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  MatrixXd Hj = Tools::CalculateJacobian(z);
  MatrixXd y = z - Hj*this.x_;
  MatrixXd S = Hj* this.P_ * Hj.transpose() + this.R_;
  MatrixXd K = this.P_ * Hj.transpose() * S.inverse();
  MatrixXd I = MatrixXd::Identity(4,4);
  this.x_ = this.x_ + K * y;
  this.P_ = (I-K*Hj) * this.P_;
}