#include "kalman_filter.h"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
	MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) 
{
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;
}

void KalmanFilter::Predict() 
{
	// new state
	x_prev_ = x_;
	x_ = F_ * x_;

	// new covariance for x
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) 
{
	VectorXd y = z - H_ * x_;
	MatrixXd H_t = H_.transpose();
	MatrixXd S = H_ * P_ * H_t + R_;
	MatrixXd K = P_ * H_t * S.inverse();

	// new state
	x_ = x_ + (K * y);

	// new covariance for x
	MatrixXd I = MatrixXd::Identity(4,4);
	P_ = ( I - K * H_ ) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z, float dt) 
{
	float pi = 22./7.f;
	//calculate h(x')
	float rho = sqrt ( x_(0) * x_(0) + x_(1) * x_(1) );	
	float phi = atan2( x_(1), x_(0) );	
	float rho_dot;
	// check for divide by zero
	if ( rho < 1e-5 )
	{
		float rho_prev = sqrt ( x_prev_(0) * x_prev_(0) + x_prev_(1) * x_prev_(1) );
		rho_dot = - rho_prev / dt;
	}
	else
		rho_dot = ( x_(0) * x_(2) + x_(1) * x_(3) ) / rho; 
	
	VectorXd h_x(3);
	h_x << rho, phi, rho_dot;

	VectorXd y = z - h_x;
	// normalize y(1) such that -pi <= y(1) <= pi
	while ( y(1) < -pi || y(1) > pi)
	{ 
		if ( y(1) > pi )
			y(1) -= 2*pi;
		if ( y(1) < -pi)
			y(1) += 2*pi;
	}
	MatrixXd H_t = H_.transpose();
	MatrixXd S = H_ * P_ * H_t + R_;
	MatrixXd K = P_ * H_t * S.inverse();

	// new state
	x_ = x_ + (K * y);

	// new covariance for x
	MatrixXd I = MatrixXd::Identity(4,4);
	P_ = ( I - K * H_ ) * P_;
}
