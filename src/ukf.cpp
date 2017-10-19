#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 30;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 30;

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
    TODO:

    Complete the initialization. See ukf.h for other member properties.

    Hint: one or more values initialized above might be wildly off...
    */
    n_x_ = 5;
    n_aug_ = 7;
    lambda_ = 3 - n_aug_;
    weights_ = VectorXd(2 * n_aug_ + 1);

    //set weights
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < 2 * n_aug_ + 1; i++) {
        weights_(i) = 0.5 / (lambda_ + n_aug_);
    }
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Make sure you switch between lidar and radar
    measurements.
    */

    /**
     * Initialization
     */
    if (!is_initialized_) {

        if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {
            x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
        } else if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            double r_, phi_, r_dot_, p_x_, p_y_;
            r_ = meas_package.raw_measurements_[0];
            phi_ = meas_package.raw_measurements_[1];
            r_dot_ = meas_package.raw_measurements_(2);
            p_x_ = r_ * cos(phi_);
            p_y_ = r_ * sin(phi_);

            x_ << p_x_, p_y_, sqrt(pow(r_dot_ * cos(phi_), 2) + pow(r_dot_ * sin(phi_), 2));

        } else {
            return;
        }

        time_us_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }


    double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;

    /**
     * Prediction
     */
    Prediction(dt);

    /**
     * Update
     */
    if (use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {
        UpdateLidar(meas_package);
    } else if (use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        UpdateRadar(meas_package);
    } else {
        return;
    }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    /**
    TODO:

    Complete this function! Estimate the object's location. Modify the state
    vector, x_. Predict sigma points, the state, and the state covariance matrix.
    */
    //create sigma points
//    MatrixXd Xsig_ = MatrixXd(n_x_, 2 * n_x_+1);
//
//    //calculate square root of P
//    MatrixXd A_ = P_.llt().matrixL();
//
//    Xsig_.col(0) << x_;
//    for(int i = 0; i < n_x_; i++){
//        Xsig_.col(i+1) << x_ + (sqrt(lambda_ + n_x_) * A_.col(i));
//        Xsig_.col(i + 1 + n_x_) << x_ - (sqrt(lambda_ + n_x_) * A_.col(i));
//    }

    /**
     * create and augment sigma points
     */
    VectorXd x_aug_;
    MatrixXd P_aug_, Xsig_aug_;

    //create augmented mean vector
    x_aug_ = VectorXd(n_aug_);

    //create augmented state covariance
    P_aug_ = MatrixXd(n_aug_, n_aug_);

    //create sigma point matrix
    Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

    x_aug_.head(n_x_) << x_;
    P_aug_.topLeftCorner(n_x_, n_x_) = P_;
    P_aug_(n_x_, n_x_) = pow(std_a_, 2);
    P_aug_(n_x_ + 1, n_x_ + 1) = pow(std_yawdd_, 2);

    //create square root matrix
    MatrixXd A_aug_ = P_aug_.llt().matrixL();
    Xsig_aug_.col(0) << x_aug_;
    for (int i = 0; i < n_aug_; i++) {
        Xsig_aug_.col(i + 1) << x_aug_ + (sqrt(lambda_ + n_aug_) * A_aug_.col(i));
        Xsig_aug_.col(i + 1 + n_aug_) << x_aug_ - (sqrt(lambda_ + n_aug_) * A_aug_.col(i));
    }

    //predict sigma points
    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        double p_x_, p_y_, v_, yaw_, yawd_, nu_a_, nu_yawdd_;
        p_x_ = Xsig_aug_(0, i);
        p_y_ = Xsig_aug_(1, i);
        v_ = Xsig_aug_(2, i);
        yaw_ = Xsig_aug_(3, i);
        yawd_ = Xsig_aug_(4, i);
        nu_a_ = Xsig_aug_(5, i);
        nu_yawdd_ = Xsig_aug_(6, i);

        double px_p_, py_p_, v_p_, yaw_p_, yawd_p_;

        //avoid division by zero
        if (fabs(yawd_) > 0.0001) {
            px_p_ = p_x_ + v_ / yawd_ * (sin(yaw_ + yawd_ * delta_t) - sin(yaw_));
            py_p_ = p_y_ + v_ / yawd_ * (-cos(yaw_ + yawd_ * delta_t) + cos(yaw_));
        } else {
            px_p_ = p_x_ + v_ * cos(yaw_) * delta_t;
            py_p_ = p_x_ + v_ * sin(yaw_) * delta_t;
        }
        px_p_ = px_p_ + (pow(delta_t, 2) * cos(yaw_) * nu_a_) / 2;
        py_p_ = py_p_ + (pow(delta_t, 2) * sin(yaw_) * nu_a_) / 2;
        v_p_ = v_ + delta_t * nu_a_;
        yaw_p_ = yaw_ + yawd_ * delta_t + (pow(delta_t, 2) * nu_yawdd_) / 2;
        yawd_p_ = yawd_ + delta_t * nu_yawdd_;

        //write predicted sigma points into right column
        Xsig_pred_(0, i) = px_p_;
        Xsig_pred_(1, i) = py_p_;
        Xsig_pred_(2, i) = v_p_;
        Xsig_pred_(3, i) = yaw_p_;
        Xsig_pred_(4, i) = yawd_p_;
    }

    //predict state mean
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }

    //predict state covariance matrix
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd x_diff_ = Xsig_pred_.col(i) - x_;

        while (x_diff_(3) > M_PI) x_diff_(3) -= 2 * M_PI;
        while (x_diff_(3) < -M_PI) x_diff_(3) += 2 * M_PI;

        P_ = P_ + weights_(i) * x_diff_ * x_diff_.transpose();
    }

}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Use lidar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the lidar NIS.
    */

    int n_z_ = 2;
    MatrixXd Zsig_, S_, Tc_, R_, K_;
    VectorXd z_, z_pred_;

    //create matrix for sigma points in measurement space
    Zsig_ = MatrixXd(n_z_, 2 * n_aug_ + 1);

    //measurement covariance matrix S
    S_ = MatrixXd(n_z_, n_z_);

    //create matrix for cross correlation Tc
    Tc_ = MatrixXd(n_x_, n_z_);

    //mean predicted measurement
    z_pred_ = VectorXd(n_z_);

    //create vector for incoming radar measurement
    z_ = meas_package.raw_measurements_;

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        double p_x_, p_y_;
        p_x_ = Xsig_pred_(0, i);
        p_y_ = Xsig_pred_(1, i);

        Zsig_(0, i) = p_x_;
        Zsig_(1, i) = p_y_;
    }

    //calculate mean predicted measurement
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred_ = z_pred_ + weights_(i) * Zsig_.col(i);
    }

    //calculate measurement covariance matrix S
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd z_diff = Zsig_.col(i) - z_pred_;

        //angle normalization
        while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

        S_ = S_ + weights_(i) * z_diff * z_diff.transpose();
    }

    R_ = MatrixXd(n_z_, n_z_);
    R_ << pow(std_laspx_, 2), 0,
            0, pow(std_laspy_, 2);

    S_ = S_ + R_;

    //calculate cross correlation matrix
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        VectorXd z_diff = Zsig_.col(i) - z_pred_;

        while (x_diff(3) > M_PI) x_diff(3) -= 2 * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2 * M_PI;

        while (z_diff(1) > M_PI) z_diff(1) -= 2 * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2 * M_PI;

        Tc_ = Tc_ + weights_(i) * x_diff * z_diff.transpose();
    }
    //calculate Kalman gain K;
    K_ = Tc_ * S_.inverse();
    //update state mean and covariance matrix
    x_ = x_ + K_ * (z_ - z_pred_);
    P_ = P_ - K_ * S_ * K_.transpose();
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Use radar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the radar NIS.
    */
    int n_z_ = 3;
    MatrixXd Zsig_, S_, Tc_, R_, K_;
    VectorXd z_, z_pred_;

    //create matrix for sigma points in measurement space
    Zsig_ = MatrixXd(n_z_, 2 * n_aug_ + 1);

    //measurement covariance matrix S
    S_ = MatrixXd(n_z_, n_z_);

    //create matrix for cross correlation Tc
    Tc_ = MatrixXd(n_x_, n_z_);

    //mean predicted measurement
    z_pred_ = VectorXd(n_z_);

    //create vector for incoming radar measurement
    z_ = meas_package.raw_measurements_;

    //transform sigma points into measurement space
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        double r_, r_dot_, phi_, p_x_, p_y_, v_, yaw_;
        p_x_ = Xsig_pred_(0, i);
        p_y_ = Xsig_pred_(1, i);
        v_ = Xsig_pred_(2, i);
        yaw_ = Xsig_pred_(3, i);

        r_ = sqrt(pow(p_x_, 2) + pow(p_y_, 2));
        phi_ = atan2(p_y_, p_x_);
        r_dot_ = (p_x_ * cos(yaw_) * v_ + p_y_ * sin(yaw_) * v_) / r_;

        Zsig_(0, i) = r_;
        Zsig_(1, i) = phi_;
        Zsig_(2, i) = r_dot_;
    }

    //calculate mean predicted measurement
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        z_pred_ = z_pred_ + weights_(i) * Zsig_.col(i);
    }

    //calculate measurement covariance matrix S
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd z_diff = Zsig_.col(i) - z_pred_;

        //angle normalization
        while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

        S_ = S_ + weights_(i) * z_diff * z_diff.transpose();
    }

    R_ = MatrixXd(n_z_, n_z_);
    R_ << pow(std_radr_, 2), 0, 0,
            0, pow(std_radphi_, 2), 0,
            0, 0, pow(std_radrd_, 2);

    S_ = S_ + R_;

    //calculate cross correlation matrix
    for (int i = 0; i < 2 * n_aug_ + 1; i++) {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        VectorXd z_diff = Zsig_.col(i) - z_pred_;

        while (x_diff(3) > M_PI) x_diff(3) -= 2 * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2 * M_PI;

        while (z_diff(1) > M_PI) z_diff(1) -= 2 * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2 * M_PI;

        Tc_ = Tc_ + weights_(i) * x_diff * z_diff.transpose();
    }
    //calculate Kalman gain K;
    K_ = Tc_ * S_.inverse();
    //update state mean and covariance matrix
    x_ = x_ + K_ * (z_ - z_pred_);
    P_ = P_ - K_ * S_ * K_.transpose();

}
