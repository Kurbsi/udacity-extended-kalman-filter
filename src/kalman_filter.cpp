#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   * TODO: predict the state
   */
    x_ = F_ * x_; //+ u;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
    VectorXd y = z - H_ * x_;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;

    // new state
    x_ = x_ + (K * y);
    P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
    
    VectorXd h = ConvertStateToPolar(x_);
  
    VectorXd y = z - h;
    while (abs(y[1]) > M_PI){
      if (y[1] < 0){
        y[1] += 2*M_PI;
      }
      else
      {
        y[1]-=2*M_PI;
      }
    }
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;

    // new state
    x_ = x_ + (K * y);
    MatrixXd I = MatrixXd::Identity(x_.size(), x_.size()); 
    P_ = (I - K * H_) * P_;
}

VectorXd KalmanFilter::ConvertStateToPolar(const VectorXd& state) {
	const float px = state(0);
	const float py = state(1);
	const float vx = state(2);
	const float vy = state(3);
  
    const float rho = sqrt(px*px + py*py);

    if(fabs(px) < 0.0001 || fabs(py) < 0.0001) {
        return Vector3d(rho, 0.0, 0.0);
    }
       
    const float phi = atan2(py,px);
//     while (abs(phi) > M_PI){
//       if (phi < 0){
//         phi += 2*M_PI;
//       }
//       else
//       {
//         phi-=2*M_PI;
//       }
//     }
  
    const float rho_dot = (px*vx + py*vy) / rho;
  
    return Vector3d(rho, phi, rho_dot);
}
