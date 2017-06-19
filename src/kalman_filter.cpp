#include "kalman_filter.h"
#include <iostream>

using namespace std;

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
  /**
  TODO:
    * predict the state
  */

    //cout << "x = Fx (no noise)" << endl;
    this->x_ = this->F_*this->x_; // + u; // u?
    MatrixXd Ft = this->F_.transpose();
    //cout << "P = F*P*F_t + Q" << endl;
    this->P_ = this->F_ * this->P_ * Ft + this->Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

    //cout << "y = z - H*x" << endl;
    VectorXd y = z - this->H_ * this->x_;
    MatrixXd Ht = this->H_.transpose();
    //cout << "S = H*P*H_t + R" << endl;
    MatrixXd S = this->H_ * this->P_ * Ht + this->R_;
    MatrixXd Si = S.inverse();
    //cout << "K = P*H_t*S_-1" << endl;
    MatrixXd K =  this->P_ * Ht * Si;

    //cout << "new state x = x + K*y" << endl;
    this->x_ = this->x_ + (K * y);
    long x_size = this->x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    //cout << "P = (I - K*H)*P" << endl;
    this->P_ = (I - K * this->H_) * this->P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  /* Same but use jacobian of F and H */

  //cout << "y = z - Hx" << endl;
  VectorXd y = z - this->H_ * this->x_;
  MatrixXd Ht = this->H_.transpose();
  //cout << "S = H*P*H_t + R" << endl;
  MatrixXd S = this->H_ * this->P_ * Ht + this->R_;
  MatrixXd Si = S.inverse();
  //cout << "K = P*H_t*S_-1" << endl;
  MatrixXd K =  this->P_ * Ht * Si;

  //cout << "new state x = x + K*y" << endl;
  this->x_ = this->x_ + (K * y);
  long x_size = this->x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  //cout << "P = (I - K*H)*P" << endl;
  this->P_ = (I - K * this->H_) * this->P_;

}
