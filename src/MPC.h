#pragma once

#include "Eigen-3.3/Eigen/Core"

#include <vector>

using namespace std;

class MPC
{
public:
	struct Solution
	{
		double steer_value;
		double throttle_value;
	};

	MPC();

	virtual ~MPC();

	// Solve the model given an initial state and polynomial coefficients.
	// Return the first actuatotions.
	Solution Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};
