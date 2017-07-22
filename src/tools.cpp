#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
VectorXd rmse(4);
rmse << 0,0,0,0;

if(estimations.size() != ground_truth.size() || ground_truth.size() == 0){
	std::cout << "Invalid estimations or ground truth table" << endl;
	return rmse;
}
for(unsigned int i=0; i<estimations.size(); i++)
{
	VectorXd residual = estimations[i] - ground_truth[i];
	residual = residual.array() * residual.array();
	rmse += residual;
}
rmse = rmse / estimations.size();
rmse = rmse.array().sqrt();
return rmse;
}// end of function

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
        MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE 

	//check division by zero
	if ((px == 0) && (py == 0))
	{
	    cout <<"Divide by zero error" << endl;
	    return Hj;
	}
	
	//compute the Jacobian matrix
	float x = sqrt(px*px + py*py);
	float x2 = x*x;
	float x3 = x2*x;
	
	Hj(0,0) = px/x;
	Hj(0,1) = py/x;
	Hj(0,2) = 0;
	Hj(0,3) = 0;
	Hj(1,0) = -py/x2;
	Hj(1,1) = px/x2;
	Hj(1,2) = 0;
	Hj(1,3) = 0;
	Hj(2,0) = py*(vx*py - vy*px)/x3;
	Hj(2,1) = px*(-vx*py + vy*px)/x3;
	Hj(2,2) = px/x;
	Hj(2,3) = py/x;
	

	return Hj;	
}
