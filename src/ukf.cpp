#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF()
{
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // initial state vector
    x_ = VectorXd(5);

    // initial covariance matrix
    P_ = MatrixXd(5, 5);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 1.5;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.5;

    //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
    //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

    /**
        TODO:
        Complete the initialization. See ukf.h for other member properties.

        Hint: one or more values initialized above might be wildly off...
    */
    is_initialized_ = false;

    //Initialize for sigma points prediction
    n_x_ = 5;
    n_aug_ = n_x_ + 2;
    lambda_ = 3 - n_aug_;
    n_sig_ = 2 * n_aug_ + 1;

    Xsig_pred_ = MatrixXd(n_x_, n_sig_);

    weights_ = VectorXd(n_sig_);
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    for (int i = 1; i < n_sig_; i++)
    {
        weights_(i) = 1 / (2 * (lambda_ + n_aug_));
    }

    //Initialize for predicted measurements
    R_radar_ = MatrixXd(3, 3);
    R_radar_ << std_radr_ * std_radr_, 0, 0,
                0, std_radphi_ * std_radphi_, 0,
                0, 0, std_radrd_ * std_radrd_;

    R_laser_ = MatrixXd(2, 2);
    R_laser_ << std_laspx_ * std_laspx_, 0,
                0, std_laspy_ * std_laspy_;

    H_laser_ = MatrixXd(2, n_x_);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
     * TODO:
     * Complete this function! Make sure you switch between lidar and radar
     * measurements.
    */
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
            double rho = meas_package.raw_measurements_[0]; // range
            double phi = meas_package.raw_measurements_[1]; // angular distance
            double rho_dot = meas_package.raw_measurements_[2]; // rate of change of rho
            double px = rho * cos(phi);
            double py = rho * sin(phi);
            double vx = rho_dot * cos(phi);
            double vy = rho_dot * sin(phi);
            double v = sqrt(vx * vx + vy * vy);
            x_ << px, py, 0, 0, 0;
            // x_ << px, py, v, phi, 0;
        } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            double px = meas_package.raw_measurements_[0];
            double py = meas_package.raw_measurements_[1];
            x_ << px, py, 0, 0, 0;
        }

        P_ << 1, 0, 0, 0, 0,
              0, 1, 0, 0, 0,
              0, 0, 10, 0, 0,
              0, 0, 0, 50, 0,
              0, 0, 0, 0, 3;

        time_us_ = meas_package.timestamp_;

        // done initializing, no need to predict or update
        is_initialized_ = true;

        return;
    }

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
    time_us_ = meas_package.timestamp_;

    Prediction(delta_t);

    /*****************************************************************************
     *  Update
     ****************************************************************************/
    if ((meas_package.sensor_type_ == MeasurementPackage::RADAR) && (use_radar_)) {
        UpdateRadar(meas_package);
    } else if ((meas_package.sensor_type_ == MeasurementPackage::LASER) && (use_laser_)) {
        UpdateLidar(meas_package);
    }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
    /**
     * TODO:
     * Complete this function! Estimate the object's location. Modify the state
     * vector, x_. Predict sigma points, the state, and the state covariance matrix.
    */
    MatrixXd Xsig_aug = GenerateSigmaPoints();
    PredictSigmaPoints(Xsig_aug, delta_t);
    PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
    /**
     * TODO:
     * Complete this function! Use lidar data to update the belief about the object' 
     * position. Modify the state vector, x_, and covariance, P_.
     * You'll also need to calculate the lidar NIS.
    */
    H_laser_ << 1, 0, 0, 0, 0,
                0, 1, 0, 0, 0;

    VectorXd z_pred = H_laser_ * x_;
    VectorXd y = meas_package.raw_measurements_ - z_pred;

    MatrixXd Ht = H_laser_.transpose();
    MatrixXd S = H_laser_ * P_ * Ht + R_laser_;

    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_laser_) * P_;

    //Calculating NIS
    NIS_laser_ = y.transpose() * S.inverse() * y;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
    /**
     * TODO:
     * Complete this function! Use radar data to update the belief about the object's
     * position. Modify the state vector, x_, and covariance, P_.
     * You'll also need to calculate the radar NIS.
    */

    /*****************************************************************************
     *  Predict Radar measurement
     ****************************************************************************/

    //measurement dimension
    int n_z = 3;

    //matrix to store sigma points in measurement space
    MatrixXd Zsig = MatrixXd(n_z, n_sig_);

    //matrix to store predicted sigma points
    VectorXd z_pred = VectorXd(n_z);

    //covariance matrix with predicted covariances
    MatrixXd S = MatrixXd(n_z, n_z);
    
    PredictRadarMeasurement(n_z, &Zsig, &z_pred, &S);

    /*****************************************************************************
     *  Update states
     ****************************************************************************/

    //Matrix to store cross correlation
    MatrixXd Tc = MatrixXd(n_x_, n_z);
    Tc.fill(0.0);
    for (int i = 0; i < n_sig_; i++)
    {
        VectorXd diff_state = Xsig_pred_.col(i) - x_;
        diff_state(3) = atan2(sin(diff_state(3)), cos(diff_state(3)));

        VectorXd diff_meas = Zsig.col(i) - z_pred;
        diff_meas(1) = atan2(sin(diff_meas(1)), cos(diff_meas(1)));

        Tc = Tc + weights_(i) * diff_state * diff_meas.transpose();
    }

    MatrixXd K = Tc * S.inverse();

    VectorXd diff_meas = meas_package.raw_measurements_ - z_pred;
    diff_meas(1) = atan2(sin(diff_meas(1)), cos(diff_meas(1)));

    x_ = x_ + K * diff_meas;

    P_ = P_ - K * S * K.transpose();

    //Calculating NIS
    NIS_radar_ = diff_meas.transpose() * S.inverse() * diff_meas;
}

/**
 *  Generate sigma points:
 *  @return Xsig_aug: Generated sigma points
 */
MatrixXd UKF::GenerateSigmaPoints() {
    VectorXd x_aug_ = VectorXd(n_aug_);
    x_aug_.head(5) = x_;
    x_aug_(5) = 0;
    x_aug_(6) = 0;

    MatrixXd P_aug_ = MatrixXd(n_aug_, n_aug_);
    P_aug_.fill(0.0);
    P_aug_.topLeftCorner(5, 5) = P_;
    P_aug_(5, 5) = std_a_ * std_a_;
    P_aug_(6, 6) = std_yawdd_ * std_yawdd_;

    //create sigma point matrix
    MatrixXd Xsig_aug = MatrixXd(n_aug_, n_sig_);

    //calculate square root of P
    MatrixXd A = P_aug_.llt().matrixL();

    Xsig_aug.col(0) = x_aug_;

    for (int i = 0; i < n_aug_; i++) {
        Xsig_aug.col(i + 1) = x_aug_ + sqrt(lambda_ + n_aug_) * A.col(i);
        Xsig_aug.col(i + 1 + n_aug_) = x_aug_ - sqrt(lambda_ + n_aug_) * A.col(i);
    }
    
    return Xsig_aug;
}

/**
 *  This function generated the sigma points in the measurement space
 *  @param Xsig_aug: Generated sigma points
 *  @param delta_t: the change in time in sections between adjacent measurements
 */
void UKF::PredictSigmaPoints(MatrixXd Xsig_aug, double delta_t) {
    //predict sigma points
    for (int i = 0; i < n_sig_; i++) {
        //extract values for better readability
        double p_x = Xsig_aug(0, i);
        double p_y = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        //predicted state values
        double px_pred, py_pred;

        //avoid division by zero
        if (fabs(yawd) > 0.001) {
            px_pred = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
            py_pred = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
        } else {
            px_pred = p_x + v * delta_t * cos(yaw);
            py_pred = p_y + v * delta_t * sin(yaw);
        }

        double v_pred = v;
        double yaw_pred = yaw + yawd * delta_t;
        double yawd_pred = yawd;

        //add noise
        px_pred = px_pred + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
        py_pred = py_pred + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
        v_pred = v_pred + nu_a * delta_t;

        yaw_pred = yaw_pred + 0.5 * nu_yawdd * delta_t * delta_t;
        yawd_pred = yawd_pred + nu_yawdd * delta_t;

        //write predicted sigma point into right column
        Xsig_pred_(0, i) = px_pred;
        Xsig_pred_(1, i) = py_pred;
        Xsig_pred_(2, i) = v_pred;
        Xsig_pred_(3, i) = yaw_pred;
        Xsig_pred_(4, i) = yawd_pred;
    }
}

/**
 *  This function generate the predicted the mean and covariance in the measurement space
 *  based on the predicted sigma points
 */
void UKF::PredictMeanAndCovariance() {
    //Predicted state mean
    x_.fill(0.0);
    for (int i = 0; i < n_sig_; i++){
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }

    //Predicted state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < n_sig_; i++) {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;
        x_diff(3) = atan2(sin(x_diff(3)), cos(x_diff(3)));
        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    }
}

/**
 *  Self explainatory, checks the rho value before the division
 *  @param: rho as denominator
 *  @return adjusted rho as dennominator
 */
double UKF::AvoidZeroDenominator(double denominator) {
    if (fabs(denominator) < 0.0001) {
        if (denominator > 0) {
            denominator += 0.0001;
        } else {
            denominator -= 0.0001;
        }
    }

    return denominator;  
}

/**
 *  This function generate the predicted radar measurement at t = k+1 with measurement data from
 *  t = k
 *  @param: n_z is the dimension for radar measurement (rho, phi, rho_dot)
 *  @param: Zsig is the collection of sigma points in measurement space
 *  @param: z_out is the collection of predicted sigma points in measurement spadce at t = k+1
 *          with measurements from t = k
 *  @param: S_out is the predicted measurement covariance matrix at t = k+1 
 *          with measurements from t = k
 */
void UKF::PredictRadarMeasurement(int n_z, 
                                  MatrixXd *Zsig_out, 
                                  VectorXd* z_out, 
                                  MatrixXd* S_out) {
    MatrixXd Zsig = MatrixXd(n_z, n_sig_);
    VectorXd z_pred = VectorXd(n_z);
    MatrixXd S = MatrixXd(n_z, n_z);

    //converting predicted state space to measurement space
    for (int i = 0; i < n_sig_; i++) {
        double p_x = Xsig_pred_(0,i);
        double p_y = Xsig_pred_(1,i);
        double v  = Xsig_pred_(2,i);
        double yaw = Xsig_pred_(3,i);

        double v_x = cos(yaw)*v;
        double v_y = sin(yaw)*v;

        double rho = sqrt(p_x*p_x + p_y*p_y);                        
        double phi = atan2(p_y,p_x);
        rho = AvoidZeroDenominator(rho);                             
        double rho_dot = (p_x*v_x + p_y*v_y ) / rho;

        Zsig(0, i) = rho;
        Zsig(1, i) = phi;
        Zsig(2, i) = rho_dot;
    }

    // Store predicted sigma points
    z_pred.fill(0.0);
    for (int i = 0; i < n_sig_; i++){
        z_pred = z_pred + weights_(i) * Zsig.col(i);
    }

    // Create predicted covariances matrix
    S.fill(0.0);
    for (int i = 0; i < n_sig_; i++){
        VectorXd z_diff = Zsig.col(i) - z_pred;
        z_diff(1) = atan2(sin(z_diff(1)), cos(z_diff(1)));
        S = S + weights_(i) * z_diff * z_diff.transpose();
    }

    //add measurement noise covariance matrix
    S = S + R_radar_;

    *Zsig_out = Zsig;
    *z_out = z_pred;
    *S_out = S;
}