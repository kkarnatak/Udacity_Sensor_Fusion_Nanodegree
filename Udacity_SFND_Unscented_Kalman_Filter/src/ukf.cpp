#include "ukf.h"
#include "Eigen/Dense"
#include <fstream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

std::fstream f;
///////////////////////////////////////////////////////////////////////////////

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() 
{
    is_initialized_ = false;
    previous_timestamp_ = 0;

    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = true;

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 5.0;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.8;

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

    /**
     * TODO: Complete the initialization. See ukf.h for other member properties.
     * Hint: one or more values initialized above might be wildly off...
     */

    // State dimension
    n_x_ = 5;

    // Augmented state dimension
    n_aug_ = 7;

    // Sigma point spreading parameter
    lambda_ = 3 - n_x_;

    // state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
    x_ = Eigen::VectorXd::Zero(n_x_);

    // state covariance matrix
    // TODO KK, if time play with the values

    P_ = MatrixXd::Zero(n_x_, n_x_);

    // Taken from the lecture code
    // Weight Metrix
    weights_ = Eigen::VectorXd(2 * n_aug_ + 1);

    // Used from lecture code
    Xsig_pred_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);

    // Measurement Noises
    R_laser_ = Eigen::MatrixXd(2, 2);
    R_laser_ << std_laspx_ * std_laspx_, 0,
    0, std_laspy_ * std_laspy_;

    R_radar_ = Eigen::MatrixXd(3, 3);
    R_radar_ <<     std_radr_ * std_radr_,  0,  0,
                0,  std_radphi_ * std_radphi_, 0,
                0,  0,  std_radrd_ * std_radrd_;
}

///////////////////////////////////////////////////////////////////////////////

UKF::~UKF() {}

///////////////////////////////////////////////////////////////////////////////

void UKF::ProcessMeasurement(MeasurementPackage meas_package) 
{
    /**
     * TODO: Complete this function! Make sure you switch between lidar and radar
     * measurements.
    */

    if (meas_package.sensor_type_ == MeasurementPackage::RADAR && !use_radar_) 
    {
        return;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER && !use_laser_) 
    {
        return;
    }
  
    // If first time
    if (!is_initialized_)
    {
        std::cout << "First time initialization: ProcessMeasurement.";

        // Switch between lidar and radar
        if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
        {
            // Get measurements
            // Convert radar from polar to cartesian coordinates and initialize state.
            double rho = meas_package.raw_measurements_[0];
            double phi = meas_package.raw_measurements_[1];
            double v = meas_package.raw_measurements_[2];
            double px = rho * cos(phi);
            double py = rho * sin(phi);
            double yawd = 0.0;

            // Push it to the state vector
            x_ << px, py, v, phi, yawd;
            std::cout << "KKLOG: RADAR Sensor > " << px << ", " << py << std::endl;
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER)
        {
            // Get measurements
            double px = meas_package.raw_measurements_[0];
            double py = meas_package.raw_measurements_[1];

            // Push it to the state vector
            x_ << px, py, 0, 0, 0;
            //std::cout << "KKLOG: LASER Sensor > " << px << ", " << py << std::endl;
        }

        // Update the timestamp and first time flag
        previous_timestamp_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }

    if ((use_radar_ && meas_package.sensor_type_ == meas_package.RADAR) ||
        (use_laser_ && meas_package.sensor_type_ == meas_package.LASER))
    {
        double delta_t = (meas_package.timestamp_ - previous_timestamp_) / TIME_STEP;
        previous_timestamp_ = meas_package.timestamp_;


        if (meas_package.sensor_type_ == MeasurementPackage::RADAR) 
        {
            //set measurement dimension, radar can measure r, phi, and r_dot
            n_z_ = 3;
        }
        else if (meas_package.sensor_type_ == MeasurementPackage::LASER) 
        {
            //set measurement dimension, laser can measure px, py
            n_z_ = 2;
        }

        //measurement covariance matrix
        S_ = Eigen::MatrixXd(n_z_, n_z_);

            //create matrix for sigma points in measurement space
        Z_sig_ = MatrixXd::Zero(n_z_, 2 * n_aug_ + 1);

        //mean predicted measurement
        z_pred_ = VectorXd::Zero(n_z_);

        Prediction(delta_t);

        //std::cout << "KKLOG: Sensor type = " << meas_package.sensor_type_ << ", Delta: "
        //          << delta_t << std::endl;


        if(meas_package.sensor_type_ == meas_package.RADAR)
        {
            UpdateRadar(meas_package);
        }   
        else if(meas_package.sensor_type_ == meas_package.LASER)
        {
            UpdateLidar(meas_package);
        }
        
        UpdateState(meas_package.raw_measurements_);
    }
}
///////////////////////////////////////////////////////////////////////////////

void UKF::GenerateSigmaPoints(double delta_t)
{
    // This method generates several points around the expected location
    // using non-linear transformation. These points will be used to get 
    // predicted points which will be used to find a gaussian interval

    // Generate Augmented Sigma Points

    //create augmented sigma points
    //lecture code used
    MatrixXd Xsig_aug = MatrixXd::Zero(n_aug_, 2 * n_aug_ + 1);
    
    //create augmented mean vector
    VectorXd x_aug = VectorXd::Zero(n_aug_);

    //create augmented state covariance
    MatrixXd P_aug = MatrixXd::Zero(n_aug_, n_aug_);

    //create augmented mean state
    x_aug.head(n_x_) = x_;

    //create augmented covariance matrix
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(n_x_, n_x_) = std_a_ * std_a_;
    P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

    // Cholesky decomposition
    MatrixXd L = P_aug.llt().matrixL();

    //create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    
    for (int i = 0; i< n_aug_; i++) 
    {
        Xsig_aug.col(i+1)        = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
    }

    // Code taken from the lecture assignment
    // Find State Sigma Points

    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {
        //extract values for better readability
        double p_x = Xsig_aug(0, i);
        double p_y = Xsig_aug(1, i);
        double v = Xsig_aug(2, i);
        double yaw = Xsig_aug(3, i);
        double yawd = Xsig_aug(4, i);
        double nu_a = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        //predicted state values
        double px_p, py_p;

        // Check and escpace divison by zero
        if (fabs(yawd) > SMALL_NUMBER)
        {
            px_p = p_x + v / yawd * (sin(yaw + yawd * delta_t) - sin(yaw));
            py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * delta_t));
        }
        else
        {
            px_p = p_x + v * delta_t * cos(yaw);
            py_p = p_y + v * delta_t * sin(yaw);
        }

        double v_p = v;
        double yaw_p = yaw + yawd * delta_t;
        double yawd_p = yawd;

        //add noise
        px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
        py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
        v_p = v_p + nu_a * delta_t;

        yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
        yawd_p = yawd_p + nu_yawdd * delta_t;

        //write predicted sigma point into right column
        Xsig_pred_(0, i) = px_p;
        Xsig_pred_(1, i) = py_p;
        Xsig_pred_(2, i) = v_p;
        Xsig_pred_(3, i) = yaw_p;
        Xsig_pred_(4, i) = yawd_p;
    }
}

///////////////////////////////////////////////////////////////////////////////

void UKF::PredictMeanAndCovariance()
{
    // Taken from lecture notes
    // The normalize angle is tricky as we cannot substract angles from the KF
    // Thus, we need to normalize the angles
    
    weights_(0) = lambda_ / (lambda_ + n_aug_);
    
    for (int i = 1; i < 2 * n_aug_ + 1; i++) 
    {
        double weight = 0.5 / (lambda_ + n_aug_);
        
        // Update weights
        weights_(i) = weight;
    }

    // State vector
    x_.fill(0.0);

    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {
        x_ = x_ + weights_(i) * Xsig_pred_.col(i);
    }

    // Predicted state covariance matrix
    P_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        // normalize angle
        x_diff(3) = remainder(x_diff(3), 2.0 * M_PI);

        // Update Predicted covariance matrix
        P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
    }
}

///////////////////////////////////////////////////////////////////////////////

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
    GenerateSigmaPoints(delta_t);
    PredictMeanAndCovariance();
}

///////////////////////////////////////////////////////////////////////////////

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */

    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {
        // measurement model
        Z_sig_(0, i) = Xsig_pred_(0, i);
        Z_sig_(1, i) = Xsig_pred_(1, i);
    }

    //mean predicted measurement
    z_pred_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {
        z_pred_ = z_pred_ + weights_(i) * Z_sig_.col(i);
    }

    S_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) 
    { 
        VectorXd z_diff = Z_sig_.col(i) - z_pred_;

        //Normalise angle
        z_diff(1) = remainder(z_diff(1), 2.0 * M_PI);

        S_ = S_ + weights_(i) * z_diff * z_diff.transpose();
    }

    // Update measurement covariance matrix
    S_ = S_ + R_laser_;

    // Calculating NIS
    std::string sensor_type = meas_package.sensor_type_ == meas_package.RADAR ? "RADAR" : "LASER";
    Eigen::VectorXd nis = (meas_package.raw_measurements_-z_pred_).transpose()*S_.inverse()*(meas_package.raw_measurements_-z_pred_);
    std::cout << sensor_type << " NIS measurement : " << nis << std::endl;
    f.open("laser.csv", std::fstream::app);
    f << nis << "\n";
    f.close();
}

///////////////////////////////////////////////////////////////////////////////

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
    // Read Measurements
    // Convert radar from polar to cartesian coordinates and initialize state.

    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {
        double p_x = Xsig_pred_(0, i);
        double p_y = Xsig_pred_(1, i);
        double vel = Xsig_pred_(2, i);
        double yaw = Xsig_pred_(3, i);
        double v_x = cos(yaw) * vel;
        double v_y = sin(yaw) * vel;

        // rho
        Z_sig_(0, i) = sqrt(p_x * p_x + p_y * p_y);

        // phi
        Z_sig_(1, i) = atan2(p_y, p_x);

        Z_sig_(2, i) = (Z_sig_(0, i) < 0.001 ? (p_x * v_x + p_y * v_y) / 0.001 : (p_x * v_x + p_y * v_y) / Z_sig_(0, i));
    }

    //mean predicted measurement
    z_pred_.fill(0.0);

    for (int i = 0; i < 2 * n_aug_ + 1; i++)
    {
        z_pred_ = z_pred_ + weights_(i) * Z_sig_.col(i);
    }

    S_.fill(0.0);
    for (int i = 0; i < 2 * n_aug_ + 1; i++) 
    { 
        VectorXd z_diff = Z_sig_.col(i) - z_pred_;

        //Normalise angle
        z_diff(1) = remainder(z_diff(1), 2.0 * M_PI);

        S_ = S_ + weights_(i) * z_diff * z_diff.transpose();
    }

    // Update measurement covariance matrix

    S_ = S_ + R_radar_;

    // Calculating NIS
    std::string sensor_type = meas_package.sensor_type_ == meas_package.RADAR ? "RADAR" : "LASER";
    Eigen::VectorXd nis = (meas_package.raw_measurements_-z_pred_).transpose()*S_.inverse()*(meas_package.raw_measurements_-z_pred_);
    std::cout << sensor_type << " NIS measurement : " << nis << std::endl;
f.open("radar.csv", std::fstream::app);
    f << nis << "\n";
    f.close();
}

///////////////////////////////////////////////////////////////////////////////

void UKF::UpdateState(const VectorXd& z) {
    // Cross correlation matrix Tc
    MatrixXd Tc = MatrixXd::Zero(n_x_, n_z_);

    for (int i = 0; i < 2 * n_aug_ + 1; i++) 
    {
        VectorXd z_diff = Z_sig_.col(i) - z_pred_;
    
        //Normalise angle
        z_diff(1) = remainder(z_diff(1), 2.0 * M_PI);

        // Calculating the state diff
        VectorXd x_diff = Xsig_pred_.col(i) - x_;

        //Normalise angle
        x_diff(3) = remainder(x_diff(3), 2.0 * M_PI);

        Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
    }

    // Calculate Kalman Gain
    MatrixXd K = Tc * S_.inverse();

    // Compute the residual
    VectorXd z_diff = z - z_pred_;

    //Normalise angle
    z_diff(1) = remainder(z_diff(1), 2.0 * M_PI);

    // Update State Mean
    x_ = x_ + K * z_diff;

    //Update Covariance Matrix
    P_ = P_ - K*S_*K.transpose();
}

///////////////////////////////////////////////////////////////////////////////