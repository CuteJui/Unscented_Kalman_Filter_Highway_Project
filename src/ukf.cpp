#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

UKF::UKF() {

  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.0;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1.5;

  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
   * End DO NOT MODIFY section for measurement noise values
   */

  // State dimension
  n_x_ = 5;

  // Augmented state dimension
  n_aug_ = 7;

  // predicted sigma points matrix
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;

  // Weights of sigma points
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_.fill(0.5 / (lambda_ + n_aug_));
  weights_(0) = lambda_ / (lambda_ + n_aug_);

  // Laser Measurement noise covariance matrix
  R_las_ = MatrixXd::Zero(2,2);
  R_las_(0,0) = std_laspx_ * std_laspx_;
  R_las_(1,1) = std_laspy_ * std_laspy_;

  // Radar Measurement noise covariance matrix
  R_rad_ = MatrixXd::Zero(3,3);
  R_rad_(0,0) = std_radr_ * std_radr_;
  R_rad_(1,1) = std_radphi_ * std_radphi_;
  R_rad_(2,2) = std_radrd_ * std_radrd_;

}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  // First measurement
  if (!is_initialized_) {
    P_.setIdentity(5,5);

    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_.fill(0.0);
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
      P_.topLeftCorner(2,2) = R_las_;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      double rhod = meas_package.raw_measurements_(2);
      x_.fill(0.0);
      x_(0) = rho * cos(phi);
      x_(1) = rho * sin(phi);
      P_(0,0) = std_radr_ * std_radr_;
      P_(1,1) = std_radr_ * std_radr_;
    }

    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  double delta_t = (meas_package.timestamp_ - time_us_) / 1.0e6;
  time_us_ = meas_package.timestamp_;
  Prediction(delta_t);

  if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER)
    UpdateLidar(meas_package);
  else if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR)
    UpdateRadar(meas_package);

}

void UKF::Prediction(double delta_t) {

  VectorXd x_aug_ = VectorXd::Zero(n_aug_);
  x_aug_.head(5) = x_;

  MatrixXd P_aug_ = MatrixXd::Zero(n_aug_, n_aug_);
  P_aug_.topLeftCorner(5, 5) = P_;
  P_aug_(5,5) = std_a_ * std_a_;
  P_aug_(6,6) = std_yawdd_ * std_yawdd_;

  MatrixXd L = P_aug_.llt().matrixL();

  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug_.col(0) = x_aug_;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug_.col(i + 1) = x_aug_ + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug_.col(i + n_aug_ + 1) = x_aug_ - sqrt(lambda_ + n_aug_) * L.col(i);
  }

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double px = Xsig_aug_(0,i);
    double py = Xsig_aug_(1,i);
    double v = Xsig_aug_(2,i);
    double yaw = Xsig_aug_(3,i);
    double yawd = Xsig_aug_(4,i);
    double nu_a = Xsig_aug_(5,i);
    double nu_yawdd = Xsig_aug_(6,i);

    if (fabs(yawd) > 0.001) {
      px += v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
      py += v / yawd * (-cos(yaw + yawd * delta_t) + cos(yaw));
      yaw += yawd * delta_t;
    }
    else {
      px += v * cos(yaw) * delta_t;
      py += v * sin(yaw) * delta_t;
    }

    Xsig_pred_(0,i) = px + 0.5 * nu_a * cos(yaw) * delta_t * delta_t;
    Xsig_pred_(1,i) = py + 0.5 * nu_a * sin(yaw) * delta_t * delta_t;
    Xsig_pred_(2,i) = v + nu_a * delta_t;
    Xsig_pred_(3,i) = yaw + 0.5 * nu_yawdd * delta_t * delta_t;
    Xsig_pred_(4,i) = yawd + nu_yawdd * delta_t;
  }

  // Prediction mean
  x_ = Xsig_pred_ * weights_;

  // Prediction covariance
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd x_diff_ = Xsig_pred_.col(i) - x_;

    // angle normalized
    while (x_diff_(3) > M_PI) x_diff_(3) -= 2.0 * M_PI;
    while (x_diff_(3) < -M_PI) x_diff_(3) += 2.0 * M_PI;

    P_ += weights_(i) * x_diff_ * x_diff_.transpose();
  }

}

void UKF::UpdateLidar(MeasurementPackage meas_package) {

  MatrixXd H = MatrixXd::Zero(2,5);
  H(0,0) = 1.0;
  H(1,1) = 1.0;
  VectorXd y = meas_package.raw_measurements_ - H * x_;
  MatrixXd S = H * P_ * H.transpose() + R_las_;
  MatrixXd K = P_ * H.transpose() * S.inverse();
  MatrixXd I = MatrixXd::Identity(5,5);

  x_ = x_ + K * y;
  P_ = (I - K * H) * P_;

  // Calculate NIS
  double NIS_las_ = y.transpose() * S.inverse() * y;
  std::ofstream log("NIS_Lidar.txt", std::ios_base::app | std::ios_base::out); // create a new text file
  log << NIS_las_;
  log << "\n";
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {

  // Sigma points in measurement space
  MatrixXd Zsig_ = MatrixXd(3, 2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    Zsig_(0,i) = sqrt(px * px + py * py);
    Zsig_(1,i) = atan2(py, px);
    Zsig_(2,i) = (px * cos(yaw) * v + py * sin(yaw) * v) / sqrt(px * px + py * py);
  }

  // Predicted Measurement Mean
  VectorXd z_ = Zsig_ * weights_;

  // Predicted Measurement Covariance
  MatrixXd S_ = MatrixXd::Zero(3,3);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig_.col(i) - z_;

    // angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2.0 * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;

    S_ += weights_(i) * z_diff * z_diff.transpose();
  }

  S_ += R_rad_;

  // Cross corelation matrix
  MatrixXd Tc = MatrixXd::Zero(5,3);

  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd x_diff_ =  Xsig_pred_.col(i) - x_;
    while (x_diff_(3) > M_PI) x_diff_(3) -= 2.0 * M_PI;
    while (x_diff_(3) < -M_PI) x_diff_(3) += 2.0 * M_PI;

    VectorXd z_diff_ = Zsig_.col(i) - z_;
    while (z_diff_(1) > M_PI) z_diff_(1) -= 2.0 * M_PI;
    while (z_diff_(1) < -M_PI) z_diff_(1) += 2.0 * M_PI;

    Tc += weights_(i) * x_diff_ * z_diff_.transpose();
  }

  // Kalman Gain
  MatrixXd K_ = Tc * S_.inverse();

  // residual
  VectorXd z_diff_ = meas_package.raw_measurements_ - z_;
  while(z_diff_(1) > M_PI) z_diff_(1) -= 2.0 * M_PI;
  while(z_diff_(1) < -M_PI) z_diff_(1) += 2.0 * M_PI;

  x_ = x_ + K_ * z_diff_;
  P_ = P_ - K_ * S_ * K_.transpose();

  // Calculate NIS
  double NIS_rad_ = z_diff_.transpose() * S_.inverse() * z_diff_;
  std::ofstream log("NIS_Radar.txt", std::ios_base::app | std::ios_base::out); // create a new text file
  log << NIS_rad_;
  log << "\n";
}
