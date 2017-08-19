#include "MPC.h"
#include "constants.h"

#include <uWS/uWS.h>

#include "json.hpp"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
using namespace std;


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

namespace
{

double constexpr mph2mps(double x) { return x * 0.44704; }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.rfind("}]");
	if (found_null != string::npos) {
		return "";
	} else if (b1 != string::npos && b2 != string::npos) {
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
	double result = 0.0;
	for (int i = 0; i < coeffs.size(); i++) {
		result += coeffs[i] * pow(x, i);
	}
	return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
	assert(xvals.size() == yvals.size());
	assert(order >= 1 && order <= xvals.size() - 1);
	Eigen::MatrixXd A(xvals.size(), order + 1);

	for (int i = 0; i < xvals.size(); i++) {
		A(i, 0) = 1.0;
	}

	for (int j = 0; j < xvals.size(); j++) {
		for (int i = 0; i < order; i++) {
			  A(j, i + 1) = A(j, i) * xvals(j);
		}
	}

	auto Q = A.householderQr();
	auto result = Q.solve(yvals);
	return result;
}

}


int main() {
	uWS::Hub h;

	// MPC is initialized here!
	MPC mpc;

	h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
	                   uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		string sdata = string(data).substr(0, length);
		cout << sdata << endl;
		if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
			string s = hasData(sdata);
			if (s != "") {
				auto j = json::parse(s);
				string event = j[0].get<string>();
				if (event == "telemetry") {
					// j[1] is the data JSON object
					vector<double> const ptsx = j[1]["ptsx"];
					vector<double> const ptsy = j[1]["ptsy"];

					double const px = j[1]["x"];
					double const py = j[1]["y"];
					double const psi = j[1]["psi"];
					double const v = mph2mps(j[1]["speed"]);

					assert(ptsx.size() == ptsy.size());

					double const cos_psi = cos(-psi);
					double const sin_psi = sin(-psi);

					vector<double> next_x_vals(ptsx.size());
					vector<double> next_y_vals(ptsy.size());

					Eigen::VectorXd xvals(ptsx.size());
					Eigen::VectorXd yvals(ptsy.size());

					for (size_t i = 0; i < ptsx.size(); ++i) {
						// Convert ptsx/y to vehicle coordinates
						double const dx = ptsx[i] - px;
						double const dy = ptsy[i] - py;

						next_x_vals[i] = dx * cos_psi - dy * sin_psi;
						next_y_vals[i] = dx * sin_psi + dy * cos_psi;

						xvals[i] = next_x_vals[i];
						yvals[i] = next_y_vals[i];
					}

					Eigen::VectorXd const coeffsVehicle = polyfit(xvals, yvals, 3);

					// The cross track error is calculated by evaluating at polynomial at
					// x, f(x) and subtracting y.
					double const cte = polyeval(coeffsVehicle, 0);
					// Due to the sign starting at 0, the orientation error is -f'(x).
					double const targetBearing =
						atan(coeffsVehicle[1]
						     + 2 * coeffsVehicle[2] * px
						     + 3 * coeffsVehicle[3] * px * px);
					double const epsi = -targetBearing;

					Eigen::VectorXd state(6);

					state << 0, 0, 0, v, cte, epsi;

					MPC::Solution const solution = mpc.Solve(state, coeffsVehicle);

					json msgJson;

					msgJson["steering_angle"] = solution.steer_value;
					msgJson["throttle"] = solution.throttle_value;

					// Display the MPC predicted trajectory
					msgJson["mpc_x"] = solution.predicted_x;
					msgJson["mpc_y"] = solution.predicted_y;

					// Display the waypoints/reference line
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;


					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					std::cout << msg << std::endl;
					// Latency
					// The purpose is to mimic real driving conditions where
					// the car does actuate the commands instantly.
					//
					// Feel free to play around with this value but should be to drive
					// around the track with 100ms latency.
					//
					// \note REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
					// SUBMITTING.
					this_thread::sleep_for(chrono::milliseconds(LATENCY_MS));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else {
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
	                   size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1) {
			res->end(s.data(), s.length());
		} else {
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
	                       char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	} else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}

	h.run();
}
