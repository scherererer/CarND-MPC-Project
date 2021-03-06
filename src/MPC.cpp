#include "MPC.h"
#include "constants.h"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
using CppAD::AD;

#include "Eigen-3.3/Eigen/Core"

using namespace std;


namespace
{
// This value of N seemed reasonable. I started with 20 but it seems to work just as well with
// less so we might as well reduce the computational cost
size_t constexpr N = 10;
// Want dt to be approximately equal to the latency (100ms) because I'm sampling the second
// time step's actuator values to handle the actuator latency. I tried setting it to exactly
// 100ms but found through experimentation that 150 was smoother.
double constexpr dt = 0.15;  ///< Delta time in seconds
double constexpr ref_v = 10; ///< Target velocity in m/s

// Start indices for each section of the unified state / actuator vector
size_t constexpr x_start     = 0;           size_t constexpr x_end     = x_start + N;
size_t constexpr y_start     = x_end;       size_t constexpr y_end     = y_start + N;
size_t constexpr psi_start   = y_end;       size_t constexpr psi_end   = psi_start + N;
size_t constexpr v_start     = psi_end;     size_t constexpr v_end     = v_start + N;
size_t constexpr cte_start   = v_end;       size_t constexpr cte_end   = cte_start + N;
size_t constexpr epsi_start  = cte_end;     size_t constexpr epsi_end  = epsi_start + N;
size_t constexpr delta_start = epsi_end;    size_t constexpr delta_end = delta_start + N - 1;
size_t constexpr a_start     = delta_end;   size_t constexpr a_end     = a_start + N - 1;

class FG_eval
{
public:
	// Fitted polynomial coefficients
	Eigen::VectorXd coeffs;
	FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

	typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

	/// \param fg A vector of the cost constraints
	/// \param vars A vector of variable values (state & actuators)
	void operator()(ADvector& fg, const ADvector& vars) {
		// fg[0] stores cost
		fg[0] = 0;

		// The part of the cost based on the reference state.
		for (size_t t = 0; t < N; ++t) {
			// Minimize cross track error
			fg[0] += CppAD::pow(vars[cte_start + t], 2);
			// Minimize orientation error
			fg[0] += CppAD::pow(vars[epsi_start + t], 2);
			// Attempt to drive at a target speed
			fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
		}

		// Minimize the use of actuators.
		for (size_t t = 0; t < N - 1; ++t) {
			// Prefer centered actuators
			fg[0] += CppAD::pow(vars[delta_start + t], 2);
			// This one, intuitively, seems a bit weird because it should be increasing cost
			// the more you throttle up. It's strange because the steady state throttle should
			// be non-zero, and it's not clear why that should incur a cost. Removing this,
			// however, seemed to cause the vehicle to oscillate more, so I've left it in.
			fg[0] += CppAD::pow(vars[a_start + t], 2);
		}

		// Minimize the change in actuation
		for (size_t t = 0; t < N - 2; ++t) {
			// Minimize the change in delta and acceleration
			fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
			fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
		}

		fg[1 + x_start] = vars[x_start];
		fg[1 + y_start] = vars[y_start];
		fg[1 + psi_start] = vars[psi_start];
		fg[1 + v_start] = vars[v_start];
		fg[1 + cte_start] = vars[cte_start];
		fg[1 + epsi_start] = vars[epsi_start];

		// The rest of the constraints
		for (size_t t = 1; t < N; ++t) {
			// The state at time t+1 .
			AD<double> x1 = vars[x_start + t];
			AD<double> y1 = vars[y_start + t];
			AD<double> psi1 = vars[psi_start + t];
			AD<double> v1 = vars[v_start + t];
			AD<double> cte1 = vars[cte_start + t];
			AD<double> epsi1 = vars[epsi_start + t];

			// The state at time t.
			AD<double> x0 = vars[x_start + t - 1];
			AD<double> y0 = vars[y_start + t - 1];
			AD<double> psi0 = vars[psi_start + t - 1];
			AD<double> v0 = vars[v_start + t - 1];
			AD<double> cte0 = vars[cte_start + t - 1];
			AD<double> epsi0 = vars[epsi_start + t - 1];

			// Only consider the actuation at time t.
			AD<double> delta0 = vars[delta_start + t - 1];
			AD<double> a0 = vars[a_start + t - 1];

			// Powers of the x value for later use
			AD<double> x0_2 = x0 * x0;
			AD<double> x0_3 = x0_2 * x0;

			AD<double> f0 = (coeffs[0])
				+ (coeffs[1] * x0)
				+ (coeffs[2] * x0_2)
				+ (coeffs[3] * x0_3);
			AD<double> psides0 =
				CppAD::atan(coeffs[1]
				            + 2 * coeffs[2] * x0
				            + 3 * coeffs[3] * x0_2);

			// Model Equations:
			// x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
			// y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
			// psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
			// v_[t+1] = v[t] + a[t] * dt
			// cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
			// epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
			fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
			fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
			fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
			fg[1 + v_start + t] = v1 - (v0 + a0 * dt);

			fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
			fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
		}
	}
};

}


///////////////////////////////////////////////////////////////////////////
MPC::MPC() {}
MPC::~MPC() {}

MPC::Solution MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
	bool ok = true;
	typedef CPPAD_TESTVECTOR(double) Dvector;

	// Set the number of model variables (includes both states and inputs).
	// For example: If the state is a 4 element vector, the actuators is a 2
	// element vector and there are 10 timesteps. The number of variables is:
	//
	// 4 * 10 + 2 * 9
	size_t constexpr n_vars = (6 * N) + (2 * (N - 1));

	static_assert (n_vars == a_end, "n_vars and a_end conflict");

	// Set the number of constraints
	size_t constexpr n_constraints = 6 * N;

	double const x = state[0];
	double const y = state[1];
	double const psi = state[2];
	double const v = state[3];
	double const cte = state[4];
	double const epsi = state[5];

	// Initial value of the independent variables.
	// SHOULD BE 0 besides initial state.
	Dvector vars(n_vars);
	for (size_t i = 0; i < n_vars; ++i) {
		vars[i] = 0;
	}
	// Set the initial variable values
	vars[x_start] = x;
	vars[y_start] = y;
	vars[psi_start] = psi;
	vars[v_start] = v;
	vars[cte_start] = cte;
	vars[epsi_start] = epsi;

	Dvector vars_lowerbound(n_vars);
	Dvector vars_upperbound(n_vars);

	// Set all non-actuators upper and lowerlimits
	// to the max negative and positive values.
	for (size_t i = 0; i < epsi_end; ++i) {
		/// \todo why aren't these +/- numeric_limits<double>::max()
		/// probably this is some reasonable bound on the map size
		vars_lowerbound[i] = -1.0e19;
		vars_upperbound[i] = 1.0e19;
	}

	// The upper and lower limits of delta are set to -25 and 25
	// degrees (values in radians).
	/// \todo Tune this
	for (size_t i = delta_start; i < delta_end; ++i) {
		vars_lowerbound[i] = -STEER_LIMIT;
		vars_upperbound[i] = +STEER_LIMIT;
	}

	// Acceleration/decceleration upper and lower limits.
	/// \todo Tune this
	for (size_t i = a_start; i < a_end; ++i) {
		vars_lowerbound[i] = -0.5;
		vars_upperbound[i] = 0.5;
	}

	// Lower and upper limits for the constraints
	// Should be 0 besides initial state.
	Dvector constraints_lowerbound(n_constraints);
	Dvector constraints_upperbound(n_constraints);

	for (size_t i = 0; i < n_constraints; i++) {
		constraints_lowerbound[i] = 0;
		constraints_upperbound[i] = 0;
	}

	// Starting values are constants so their constraints have a fixed range
	constraints_lowerbound[x_start] = x;
	constraints_lowerbound[y_start] = y;
	constraints_lowerbound[psi_start] = psi;
	constraints_lowerbound[v_start] = v;
	constraints_lowerbound[cte_start] = cte;
	constraints_lowerbound[epsi_start] = epsi;

	constraints_upperbound[x_start] = x;
	constraints_upperbound[y_start] = y;
	constraints_upperbound[psi_start] = psi;
	constraints_upperbound[v_start] = v;
	constraints_upperbound[cte_start] = cte;
	constraints_upperbound[epsi_start] = epsi;

	// object that computes objective and constraints
	FG_eval fg_eval(coeffs);

	//
	// NOTE: You don't have to worry about these options
	//
	// options for IPOPT solver
	std::string options;
	// Uncomment this if you'd like more print information
	options += "Integer print_level  0\n";
	// NOTE: Setting sparse to true allows the solver to take advantage
	// of sparse routines, this makes the computation MUCH FASTER. If you
	// can uncomment 1 of these and see if it makes a difference or not but
	// if you uncomment both the computation time should go up in orders of
	// magnitude.
	options += "Sparse  true        forward\n";
	options += "Sparse  true        reverse\n";
	// NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
	// Change this as you see fit.
	options += "Numeric max_cpu_time          0.5\n";

	// place to return solution
	CppAD::ipopt::solve_result<Dvector> solution;

	// solve the problem
	CppAD::ipopt::solve<Dvector, FG_eval>(
		options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
		constraints_upperbound, fg_eval, solution);

	// Check some of the solution values
	ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

	// Cost
	auto cost = solution.obj_value;
	std::cout << "Cost " << cost << std::endl;

	// Return the first actuator values. The variables can be accessed with
	// `solution.x[i]`.
	Solution s;

	// Steering solution is reversed to match steering angle, and normalized by the max angle
	// to match command format
	//
	// For both steering and throttle, skip ahead to the second command to handle latency of
	// actuator. Latency ~= dt so we can get away with this.
	s.steer_value = -solution.x[delta_start + 1] / STEER_MAX;
	s.throttle_value = solution.x[a_start + 1];

	for (size_t i = x_start; i < x_end; ++i)
		s.predicted_x.push_back (solution.x[i]);
	for (size_t i = y_start; i < y_end; ++i)
		s.predicted_y.push_back (solution.x[i]);

	return s;
}
