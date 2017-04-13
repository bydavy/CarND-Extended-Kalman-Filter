#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
  innerUpdate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Enforce phi normalization
  if(z(1) > fabs(M_PI)) {
    std::cout << "UpdateEKF () - Error - Phi needs to be normalized [-pi,pi]" << std::endl;
    return;
  }

	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);
	float c1 = sqrt(px*px+py*py);

	//check division by zero
  if(fabs(c1) < 0.0001){
    std::cout << "UpdateEKF () - Error - Division by zero" << std::endl;
    return;
  }

	VectorXd h_ = VectorXd(3);
	h_ << c1,
	      atan(py/px),
				(px*vx+py*vy)/c1;

	VectorXd y = z - h_;
  innerUpdate(y);
}

void KalmanFilter::innerUpdate(const Eigen::VectorXd &y) {
  MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
