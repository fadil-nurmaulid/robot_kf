#pragma once
#include <Eigen/Dense>

class KalmanFilter {
public:
  Eigen::VectorXd x;
  Eigen::MatrixXd P, F, Q, H, R;

  KalmanFilter(int n, int m) {
    x = Eigen::VectorXd::Zero(n);
    P = Eigen::MatrixXd::Identity(n,n);
    F = Eigen::MatrixXd::Identity(n,n);
    Q = Eigen::MatrixXd::Identity(n,n) * 0.01;
    H = Eigen::MatrixXd::Zero(m,n);
    R = Eigen::MatrixXd::Identity(m,m) * 0.05;
  }

  void predict() {
    x = F * x;
    P = F * P * F.transpose() + Q;
  }

  void update(const Eigen::VectorXd& z) {
    Eigen::VectorXd y = z - H * x;
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();
    x = x + K * y;
    P = (Eigen::MatrixXd::Identity(x.size(), x.size()) - K * H) * P;
  }
};
