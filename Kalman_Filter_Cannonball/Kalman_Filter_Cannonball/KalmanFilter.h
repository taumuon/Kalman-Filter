#pragma once

#include <array>

#include "Eigen/Dense"

class KalmanFilter
{
private:
	// TODO: Eigen::Matrix<double, n, n> where n is a template parameter
	Eigen::Matrix4d _a;
	// Eigen::Matrix<double, 4, 1> _b;
	Eigen::Matrix4d _b;
	Eigen::Matrix4d _q; // process covariance
	Eigen::Matrix4d _r; // measurement covariance
	Eigen::Matrix4d _h; // observation matrix
	Eigen::Matrix<double, 4, 1> _state_estimate;
	Eigen::Matrix4d _prob_estimate;
public:
	KalmanFilter(Eigen::Matrix4d a,
				 //Eigen::Matrix<double, 4, 1> b,
				 Eigen::Matrix4d b,
				 Eigen::Matrix4d q,
				 Eigen::Matrix4d r,
				 Eigen::Matrix4d h,
				 Eigen::Matrix<double, 4, 1> state_estimate);

	// TODO: make the control a general vector
	void measurement(std::array<double, 4> measured_state, Eigen::Matrix<double, 4, 1> u);

	Eigen::Matrix<double, 4, 1> get_state_estimate();
};

