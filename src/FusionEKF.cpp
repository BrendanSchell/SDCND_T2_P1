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
  R_laser_ << 0.0225, 0,
              0, 0.0225;
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  /**
  TODO:
    * Finish initializing the FusionEKF.
  */
  H_laser_ << 1, 0, 0, 0,
           0, 1, 0, 0;
  ekf_.P_ = MatrixXd(4,4);
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.x_ = VectorXd(4);
  ekf_.F_ = MatrixXd(4,4);
  ekf_.F_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
  nax2 = 8;
  nay2 = 8;
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
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0, 0, 0, 0;
    ekf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;
    previous_timestamp_ = measurement_pack.timestamp_;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float x_cart = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
      float y_cart = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
    if (x_cart == 0 or y_cart == 0){
        return;
      }

      ekf_.x_ << x_cart, y_cart, 0, 0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }

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
   */
   float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
   ekf_.F_ << 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0,
             0, 0, 0, 1;
  float dt2 = pow(dt,2);
  float dt3 = pow(dt,3)/2;
  float dt4 = pow(dt,4)/4;
  ekf_.Q_ <<  dt4 * nax2, 0, dt3 * nax2, 0,
          0, dt4 * nay2, 0, dt3 * nay2,
          dt3 * nax2, 0, dt2 * nax2, 0,
          0, dt3 * nay2, 0, dt2 * nay2;

  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.R_ = R_radar_;    
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
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
