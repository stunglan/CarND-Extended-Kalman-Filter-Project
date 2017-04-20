

#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;
  
  previous_timestamp_ = 0;
  
  
  
  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
  0, 0.0225;
  
  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
  0, 0.0009, 0,
  0, 0, 0.09;
  
  
  /**
   TODO:
   * Finish initializing the FusionEKF.
   * Set the process and measurement noises
   */
  
  // Initialise remaining variables in FusionEKF
  // H matrix to project laser from a 2D observation space to a 4D state in lectures
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  // need to allocate/initialise  Kalman filter variables in ekf
  // allocate memory
  // using the Init method in the kalman filter class
  
  Eigen::VectorXd x_ = VectorXd(4);; // state vector
  Eigen::MatrixXd P_ = MatrixXd(4, 4);; // state covariance matrix
  Eigen::MatrixXd F_ = MatrixXd(4,4);; // state transistion matrix
  Eigen::MatrixXd Q_ = MatrixXd(4,4); // process covariance matrix
  Eigen::MatrixXd H_; // allocated for radar or laser // measurement matrix
  Eigen::MatrixXd R_; // allocated in  // measurement covariance matrix
  
  
  ekf_.Init(x_, P_, F_, H_, R_, Q_);
  
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  
  
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
     TODO:
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * Remember: you'll need to convert radar from polar to cartesian coordinates.
     */
    
    
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ << 1, 1, 1, 1;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
       Convert radar from polar to cartesian coordinates and initialize state.
       */
      // using the cosine and sine of position and speed
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rhodot = measurement_pack.raw_measurements_[2];
      
      ekf_.x_ << rho * cos(phi), rho * sin(phi), rhodot * cos(phi), rhodot * sin(phi);
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
       Initialize state.
       */
      // using the position
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1],0,0;
      
    }
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
  
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  
  /**
   TODO:
   * Update the state transition matrix F according to the new elapsed time.
   - Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  
  
  // specified in the project description
  float noise_ax = 9.0;
  float noise_ay = 9.0;
  
  //  using the lectures and formula document
  ekf_.F_ <<   1, 0, dt, 0,
              0, 1, 0, dt,
              0, 0, 1, 0,
              0, 0, 0, 1;
  
  //2. Set the process covariance matrix Q - code from quiz

  float dt_2 = dt*dt;
  float dt_3 = dt_2*dt;
  float dt_4 = dt_3*dt;
  
  
  ekf_.Q_ <<    dt_4*noise_ax/4.0,         0,                                  dt_3*noise_ax/2.0,        0,
  0,                              dt_4*noise_ay/4.0,             0,                            dt_3*noise_ay/2.0,
  dt_3*noise_ax/2.0,         0,                                  dt_2*noise_ax,           0,
  0,                              dt_3*noise_ay/2.0,             0,                            dt_2*noise_ay;
  
  
  
  
  if (dt >= 0.00001) {
    ekf_.Predict();
  }
  

  
  
  /*****************************************************************************
   *  Update
   ****************************************************************************/
  
  /**
   TODO:
   * Use the sensor type to perform the update step.
   * Update the state and covariance matrices.
   */
  
  // using the appropriate update mathon dependent on the sensor
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    if (!ekf_.H_.isZero()){
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }
  
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}



