#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

    ///* initially set to false, set to true in first call of ProcessMeasurement
    bool is_initialized_;

    ///* if this is false, laser measurements will be ignored (except for init)
    bool use_laser_;

    ///* if this is false, radar measurements will be ignored (except for init)
    bool use_radar_;

    ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    VectorXd x_;

    ///* state covariance matrix
    MatrixXd P_;

    ///* predicted sigma points matrix
    MatrixXd Xsig_pred_;

    ///* time when the state is true, in us
    long long time_us_;

    ///* Process noise standard deviation longitudinal acceleration in m/s^2
    double std_a_;

    ///* Process noise standard deviation yaw acceleration in rad/s^2
    double std_yawdd_;

    ///* Laser measurement noise standard deviation position1 in m
    double std_laspx_;

    ///* Laser measurement noise standard deviation position2 in m
    double std_laspy_;

    ///* Radar measurement noise standard deviation radius in m
    double std_radr_;

    ///* Radar measurement noise standard deviation angle in rad
    double std_radphi_;

    ///* Radar measurement noise standard deviation radius change in m/s
    double std_radrd_;

    ///* Weights of sigma points
    VectorXd weights_;

    ///* State dimension
    int n_x_;

    ///* Augmented state dimension
    int n_aug_;

    ///* Sigma point spreading parameter
    double lambda_;

    ///* Calculated NIS value
    double NIS_laser_, NIS_radar_;


    /**
     * Constructor
     */
    UKF();

    /**
     * Destructor
     */
    virtual ~UKF();

    /**
     * Normalize angles
     * @param angle Angle to be normalized
     */
    void AngleNormalization(double &angle);

    /**
     * Calculate the Covariance
     * @param Cov_ The covariance matrix that needs to be calculated
     * @param Sig_points_ Sigma points
     * @param state_ state vector
     * @param is_prediction_ Check if process model(true) or measurement space(false) is used
     */
    void CalculateCovariance(MatrixXd &Cov_, MatrixXd Sig_points_, VectorXd state_, bool is_prediction_);

    /**
     * Calculate the Cross-Correlation
     * @param Tc_ Cross-correlation matrix
     * @param Zsig_ Sigma points in measuremnt space
     * @param z_pred_ state vector
     */
    void CalculateCrossCorrelation(MatrixXd &Tc_, MatrixXd Zsig_, VectorXd z_pred_);

    /**
     * Calculate the NIS
     * @param NIS_type_ Laser or Radar
     * @param res_ Calculated difference between state and predicted state
     * @param Cov_ Converiance matrix
     */
    void CalculateNIS(double &NIS_type_, VectorXd res_, MatrixXd Cov_);

    /**
     * ProcessMeasurement
     * @param meas_package The latest measurement data of either radar or laser
     */
    void ProcessMeasurement(MeasurementPackage meas_package);

    /**
     * Prediction Predicts sigma points, the state, and the state covariance
     * matrix
     * @param delta_t Time between k and k+1 in s
     */
    void Prediction(double delta_t);

    /**
     * Updates the state and the state covariance matrix using a laser measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateLidar(MeasurementPackage meas_package);

    /**
     * Updates the state and the state covariance matrix using a radar measurement
     * @param meas_package The measurement at k+1
     */
    void UpdateRadar(MeasurementPackage meas_package);
};

#endif /* UKF_H */
