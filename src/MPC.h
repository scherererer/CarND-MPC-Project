#pragma once

#include "Eigen-3.3/Eigen/Core"

#include <vector>

using namespace std;

class MPC
{
public:
	MPC();

	virtual ~MPC();

	// Solve the model given an initial state and polynomial coefficients.
	// Return the first actuatotions.
	vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};
