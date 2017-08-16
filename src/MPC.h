#pragma once

#include "Eigen-3.3/Eigen/Core"

#include <vector>

class MPC
{
public:
	struct Solution
	{
		double steer_value;
		double throttle_value;

		std::vector<double> predicted_x;
		std::vector<double> predicted_y;
	};

	MPC();

	virtual ~MPC();

	// Solve the model given an initial state and polynomial coefficients.
	// Return the first actuatotions.
	Solution Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};
