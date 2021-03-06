// Kalman_Filter_Cannonball.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include <array>
#include <cmath>
#include <iostream>
#include <random>

#include "Eigen/Dense"

#include "CannonBallSystem.h"
#include "KalmanFilter.h"

using State = std::array<double, 4>;

int main()
{
	auto delta_time = 0.1;

	CannonBallSystem cannonBallSystem(delta_time);

	auto initial_state = cannonBallSystem.get_initial_state();
	auto A = cannonBallSystem.AMatrix();
	auto B = cannonBallSystem.BMatrix();

	Eigen::Matrix<double, 1, 1> u;
	u(0, 0) = -9.81;

	std::default_random_engine generator;
	std::normal_distribution<double> distribution(0.0, 1.0);

	const double noise_amplitude = 10.0;

	// process covariance
	Eigen::Matrix4d Q;
	Q.setZero();

	// measurement covariance
	Eigen::Matrix4d R;
	R.setZero();
	R(0, 0) = 0.2; R(1, 1) = 0.2; R(2, 2) = 0.2; R(3, 3) = 0.2;

	// observation matrix
	Eigen::Matrix4d H;
	H.setIdentity();

	auto state = initial_state;
	Eigen::Matrix<double, 4, 1> initial_state_measured;
	initial_state_measured[0] = initial_state[0];
	initial_state_measured[1] = initial_state[1];
	initial_state_measured[2] = 300.0;
	initial_state_measured[3] = initial_state[3];

	KalmanFilter<4, 1> kalman_filter(A, B, Q, R, H, initial_state_measured);

	for (int x = 0; x < 145; ++x)
	{
		Eigen::Vector4d state_vector(state.data());
		auto updated_state = (A * state_vector) + (B * u);
		state[0] = updated_state[0]; state[1] = updated_state[1]; state[2] = updated_state[2]; state[3] = updated_state[3];

		auto x_measured = state[0] + (noise_amplitude * distribution(generator));
		auto y_measured = state[2] + (noise_amplitude * distribution(generator));

		auto estimated_state = kalman_filter.get_state_estimate();

		Eigen::Matrix<double, 4, 1> measurement_vector;
		measurement_vector(0, 0) = x_measured;
		measurement_vector(1, 0) = state[1];
		measurement_vector(2, 0) = y_measured;
		measurement_vector(3, 0) = state[3];
		kalman_filter.measurement(measurement_vector, u);

		std::cout << state[0] << " " << state[2] << " "
			      << x_measured << " " << y_measured << " "
			      << estimated_state[0] << " " << estimated_state[2] << std::endl;
	}

    return 0;
}

