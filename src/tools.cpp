#include <iostream>
#include "tools.h"
#define p_epsilon 0.00001

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) 
{
  /**
    * RMSE CALCULATION....!!
  */

	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	if(estimations.size() == 0 || estimations.size() != ground_truth.size())
	{
		cout << "Invalid estimations or ground ground_truth values" << endl;
		return rmse;
	}

	for (int i =0; i < estimations.size(); i++)
	{
		VectorXd residual_error;
		residual_error = (estimations[i] - ground_truth[i]);

		residual_error = residual_error.array()*residual_error.array();

		rmse += residual_error;
	}

	rmse = rmse/estimations.size();

	rmse = rmse.array().sqrt();

	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) 
{
  /**
    * JACOBIAN CALCULATION....!!
  */
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	MatrixXd Hj(3, 4);

	float d1 = (px*px + py*py);

	if(d1 < p_epsilon){
		d1 = p_epsilon;
	}

	float d2 = sqrt(d1);
	float d3 = d2 * d1;

	Hj << px/d2, py/d2, 0, 0,
			-py/d1, px/d1, 0, 0,
				py*(vx*py - vy*px)/d3, px*(vy*px-vx*py)/d3, px/d2, py/d2;


	return Hj;
}
