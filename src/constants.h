#pragma once

int constexpr LATENCY_MS = 100; ///< Latency in milliseconds
double constexpr LATENCY = LATENCY_MS / 1000.0; ///< Latency in seconds

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
double constexpr Lf = 2.67;

/// \brief Maximum steering angle
double constexpr STEER_MAX = 25.0 * M_PI / 180.0;
/// \brief Limit of steering angle
double constexpr STEER_LIMIT = STEER_MAX;
