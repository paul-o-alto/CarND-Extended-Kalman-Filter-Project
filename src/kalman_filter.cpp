#include "kalman_filter.h"

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

    //x_p = F*x + u;
    //P_p = F*P*F.transpose() + Q;

    this->x_ = this->F_ * this->x_; // + u; // u?
    MatrixXd Ft = this->F_.transpose();
    this->P_ = this->F_ * this->P_ * Ft + this->Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

    //y = z - H*x_p;
    //S = H*P_p*H.transpose() + R;
    //K = P_p*H.transpose()*S.inverse();
    //x = x_p + K*y;
    //P = (I - K*H)*P_p;

    VectorXd y = z - this->H_ * this->x_;
    MatrixXd Ht = this->H_.transpose();
    MatrixXd S = this->H_ * this->P_ * Ht + this->R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  this->P_ * Ht * Si;

    //new state
    this->x_ = this->x_ + (K * y);
    long x_size = this->x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    this->P_ = (I - K * this->H_) * this->P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  /* Same but use jacobian of F and H */
  //MatrixXd F_j = CalculateJacobian(F_) //const VectorXd& x_state)
  //MatrixXd H_j = CalculateJacobian(H_);

  VectorXd y = z - this->H_ * this->x_;
  MatrixXd Ht = this->H_.transpose();
  MatrixXd S = this->H_ * this->P_ * Ht + this->R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  this->P_ * Ht * Si;

  //new state
  this->x_ = this->x_ + (K * y);
  long x_size = this->x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  this->P_ = (I - K * this->H_) * this->P_;

}
