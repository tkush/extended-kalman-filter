#include <iostream>
#include "tools.h"
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
	const vector<VectorXd> &ground_truth) 
{

	// Setup and initialize an RMSE to be returned in case of errors
	VectorXd err_rmse(4);
	err_rmse << -1, -1, -1, -1;


	// Check if the estimation and ground truth are of same size
	if ( estimations.size() != ground_truth.size() )
	{
		cout << "[Error] Estimations and Ground truth measurements are of different size!" << endl;
		cout << "Cannot calculate RMSE. RMSE set to -1. " << endl;

		return err_rmse;
	}
	else if ( estimations.size() == 0 )
	{
		cout << "[Error] Estimations are of size 0!" << endl;
		cout << "Cannot calculate RMSE. RMSE set to -1." << endl;

		return err_rmse;
	}

	// Setup the RMSE vector and initialize to 0
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// Cumulative squared residuals
	for (int i=0;i<estimations.size();i++)
	{
		VectorXd residual = estimations[i] - ground_truth[i];
		residual = residual.array() * residual.array();
		rmse += residual;
	}

	// Calculate the mean
	rmse /= estimations.size();

	// Calculate the square root
	rmse = rmse.array().sqrt();

	//return
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) 
{

	// Setup the Jacobian matrix
	MatrixXd Hj(3,4);
	Hj << 0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;

	// extract state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// Pre-compute some quantities
	float px_py_sq = px*px + py*py;

	// Check for division by zero
	if ( px_py_sq < 1e-5 )
	{
		cout << "[Error] Divide by zero encountered while calculating Jacobian!" << endl;
		cout << "Returning a zero Jacobian." << endl;
		return Hj;
	}

	//compute the Jacobian matrix
	float div_1 = pow(px_py_sq, 0.5f);
	float div_2 = pow(px_py_sq, 1.5f); 

	Hj(0,0) = px/div_1;
	Hj(0,1) = py/div_1;
	Hj(0,2) = 0;
	Hj(0,3) = 0;

	Hj(1,0) = -py/px_py_sq;
	Hj(1,1) = px/px_py_sq;
	Hj(1,2) = 0;
	Hj(1,3) = 0;

	Hj(2,0) = py*(vx*py-vy*px)/div_2;
	Hj(2,1) = px*(vy*px-vx*py)/div_2;
	Hj(2,2) = px/div_1;
	Hj(2,3) = py/div_1;

	return Hj;
}
