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

	//measurement function - laser
	H_laser_ << 1, 0, 0, 0,
		0, 1, 0, 0;

	ekf_.Q_ = MatrixXd(4,4);
	ekf_.P_ = MatrixXd(4,4);
	ekf_.F_ = MatrixXd(4,4);
	ekf_.x_ = VectorXd(4);
	ekf_.x_prev_ = VectorXd(4);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack, int type) {


	/*****************************************************************************
	*  Initialization
	****************************************************************************/
	if (!is_initialized_) 
	{
		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
		{
			float rho = measurement_pack.raw_measurements_(0);
			float phi = measurement_pack.raw_measurements_(1);
			float rho_dot = measurement_pack.raw_measurements_(2);
			float px, py, vx, vy;

			if ( rho < 1e-5 )
			{
				// target is very close to radar
				// initialize px, py to be target rmse
				px = TARGET_RMSE_P;
				py = TARGET_RMSE_P;
				vx = 0;
				vy = 0;
			}		
			else
			{
				px = rho * cos (phi);
				py = rho * sin (phi);
				vx = 0; // rho_dot is rate of rho, not target velocity
				vy = 0; // rho_dot is rate of rho, not target velocity
			}
			ekf_.x_ << px, py, vx, vy;
		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
		{
			// raw measurements contains px, py. Initialize vx, vy to 0?
			ekf_.x_ << measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1), 0, 0;
		}

		previous_timestamp_ = measurement_pack.timestamp_;
		ekf_.P_ << 1, 0, 0, 0,
			       0, 1, 0, 0,
			       0, 0, 1000, 0,
			       0, 0, 0, 1000;
		ekf_.F_ = MatrixXd::Identity(4,4);

		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}

	/*****************************************************************************
	*  Prediction
	****************************************************************************/
	if ( ( type == 0 && measurement_pack.sensor_type_ == MeasurementPackage::LASER ) ||
		 ( type == 1 && measurement_pack.sensor_type_ == MeasurementPackage::RADAR ) ||
		 type == 2 )
	{
		// elapsed time
		float dt = measurement_pack.timestamp_ - previous_timestamp_;
		dt /= 1.e6;
		previous_timestamp_ = measurement_pack.timestamp_;

		// do not make a prediction if dt is small
		if ( dt > 1e-5 )
		{
			// Update state transition matrix F
			ekf_.F_(0,2) = dt;
			ekf_.F_(1,3) = dt;

			// Update the process noise covariance matrix
			float noise_ax = 9;
			float noise_ay = 9;
			ekf_.Q_ << dt*dt*dt*dt*noise_ax/4, 0, dt*dt*dt*noise_ax/2, 0,
				0, dt*dt*dt*dt*noise_ay/4, 0, dt*dt*dt*noise_ay/2,
				dt*dt*dt*noise_ax/2, 0, dt*dt*noise_ax, 0,
				0, dt*dt*dt*noise_ay/2, 0, dt*dt*noise_ay;

			// predict
			ekf_.Predict();
		}

		/*****************************************************************************
		*  Update
		****************************************************************************/

		if ( ( type == 2 || type == 1 ) && measurement_pack.sensor_type_ == MeasurementPackage::RADAR ) 
		{
			Tools jacobian;
			Hj_ = jacobian.CalculateJacobian( ekf_.x_ );
			ekf_.H_ = Hj_;
			ekf_.R_ = R_radar_;
			ekf_.UpdateEKF( measurement_pack.raw_measurements_, dt );
		} 
		else if ( type == 2 || type == 0 )
		{
			ekf_.H_ = H_laser_;
			ekf_.R_ = R_laser_;
			ekf_.Update( measurement_pack.raw_measurements_ );
		}
	}

		// print the output
		/*cout << "x_ = " << ekf_.x_ << endl;
		cout << "P_ = " << ekf_.P_ << endl;*/
}
