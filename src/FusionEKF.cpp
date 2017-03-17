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

  /**
  TODO:
    * Finish initializing the FusionEKF.
  */
  P_ = MatrixXd(4,4);
  Q_ = MatrixXd(4,4);
  x_ = VectorXd(4);
  noise_ax = 5;
  noise_ay = 5;
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
    ekf._P_ = 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0
              0, 0, 0, 1000;
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
   ekf_.F_ = 1, 0, dt, 0,
             0, 1, 0, dt,
             0, 0, 1, 0
             0, 0, 0, 1;

    ekf_.Q_ <<  pow(dt, 4) / 4 * noise_ax, 0, pow(dt, 3) / 2 * noise_ax, 0,
            0, pow(dt, 4) / 4 * noise_ay, 0, pow(dt, 3) / 2 * noise_ay,
            pow(dt, 3) / 2 * noise_ax, 0, pow(dt, 2) * noise_ax, 0,
            0, pow(dt, 3) / 2 * noise_ay, 0, pow(dt, 2) * noise_ay;

    ekf_.Predict();

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
    
    ekf_.UpdateEKF(z);
  } else {
    // Laser updates
    ekf_.Update(z);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
