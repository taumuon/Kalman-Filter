#pragma once

#include <array>

#include "Eigen/Dense"

using State = std::array<double, 4>;

class CannonBallSystem
{
private:
	Eigen::Matrix4d _a;
	// Eigen::Matrix<double, 4, 1> _b;
	Eigen::Matrix4d _b;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	CannonBallSystem(double delta_time);

	Eigen::Matrix<double, 4, 1> get_initial_state();

	Eigen::Matrix4d AMatrix() { return _a; }
	//Eigen::Matrix<double, 4, 1> BMatrix() { return _b; }
	Eigen::Matrix4d BMatrix() { return _b; }
};

