#include "stdafx.h"

#include <cmath>

#include "CannonBallSystem.h"

using State = std::array<double, 4>;

CannonBallSystem::CannonBallSystem(double delta_time)
{
	// A and B matrices in 'explicit discrete time invariant form'
	// https://en.wikipedia.org/wiki/State-space_representation
	_a.setIdentity();
	_a(0, 1) = delta_time;
	_a(2, 3) = delta_time;

	_b.setZero();
	_b(3, 0) = delta_time;
}

Eigen::Matrix<double, 4, 1> CannonBallSystem::get_initial_state()
{
	auto angle = 45.0;
	auto muzzle_velocity = 100.0;

	auto pi = M_PI;
	auto initial_velocity_x = muzzle_velocity * cos((angle * pi / 180.0));
	auto initial_velocity_y = muzzle_velocity * sin((angle * pi / 180.0));
	auto initial_location_x = 0.0;
	auto initial_location_y = 0.0;

	Eigen::Matrix<double, 4, 1> state;
	state[0] = initial_location_x;
	state[1] = initial_velocity_x;
	state[2] = initial_location_y;
	state[3] = initial_velocity_y;
	return state;
}

