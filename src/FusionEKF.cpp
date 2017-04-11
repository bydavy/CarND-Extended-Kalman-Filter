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

  //measurement matrix - laser
	H_laser_ << 1, 0, 0, 0,
			  0, 1, 0, 0;

	//create a 4D state vector, we don't know yet the values of the x state
	VectorXd x_ = VectorXd(4);

  //state covariance matrix P
	MatrixXd P_ = MatrixXd(4, 4);
	P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 1000, 0,
			  0, 0, 0, 1000;

	//measurement covariance (we don't know yet if it's laser or radar)
	MatrixXd R_ = R_laser_;

	//measurement matrix
  MatrixXd H_ = H_laser_;

	//the initial transition matrix
	MatrixXd F_ = MatrixXd(4, 4);
	F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;

  //the process covariance matrix
  MatrixXd Q_ = MatrixXd(4, 4);

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
    // first measurement
    cout << "EKF: " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      // TODO: Rho dot could be used to compute the initial velocity
      float rho_dot =  measurement_pack.raw_measurements_[2];
      float px = rho * cos(phi);
      float py = rho * sin(phi);
      ekf_.x_ << px, py, 0, 0;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    } else {
      std::cout << "ProcessMeasurement () - Error - Unknown sensor type" << std::endl;
      return;
    }

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  int noise_ax = 9;
  int noise_ay = 9;

  //set the process covariance matrix Q
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // measurement noise - radar
    ekf_.R_ = R_radar_;
    // Update all partial derivatives
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    // measurement matrix - radar
    ekf_.H_ = Hj_;
    // Radar updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else if(measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // measurement noise - laser
    ekf_.R_ = R_laser_;
    // measurement matrix - laser
    ekf_.H_ = H_laser_;
    // Laser updates
    ekf_.Update(measurement_pack.raw_measurements_);
  } else {
    std::cout << "ProcessMeasurement () - Error - Unknown sensor type" << std::endl;
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
