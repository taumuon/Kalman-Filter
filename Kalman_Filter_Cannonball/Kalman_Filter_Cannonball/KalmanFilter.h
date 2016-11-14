#pragma once

#include <array>

#include "Eigen/Dense"

template <unsigned int N, unsigned int M>
class KalmanFilter
{
private:
	Eigen::Matrix<double, N, M> _a;
	// Eigen::Matrix<double, 4, 1> _b;
	Eigen::Matrix<double, N, M> _b;
	Eigen::Matrix<double, N, N> _q; // process covariance
	Eigen::Matrix<double, N, N> _r; // measurement covariance
	Eigen::Matrix<double, N, N> _h; // observation matrix
	Eigen::Matrix<double, N, 1> _state_estimate;
	Eigen::Matrix<double, N, N> _prob_estimate;
public:
	KalmanFilter(Eigen::Matrix4d a,
		//Eigen::Matrix<double, 4, 1> b,
		Eigen::Matrix<double, N, M> b,
		Eigen::Matrix<double, N, N> q,
		Eigen::Matrix<double, N, N> r,
		Eigen::Matrix<double, N, N> h,
		Eigen::Matrix<double, N, 1> state_estimate)
	  : _a(a),
		_b(b),
		_q(q),
		_r(r),
		_h(h),
		_state_estimate(state_estimate)
	{
		_prob_estimate.setIdentity();
	}

	void measurement(std::array<double, N> measured_state, Eigen::Matrix<double, N, 1> u)
	{
		Eigen::Matrix<double, N, 1> measured_state_matrix;
		measured_state_matrix(0, 0) = measured_state[0];
		measured_state_matrix(1, 0) = measured_state[1];
		measured_state_matrix(2, 0) = measured_state[2];
		measured_state_matrix(3, 0) = measured_state[3];

		// prediction step
		auto predicted_state_estimate = (_a * _state_estimate) + (_b * u);
		auto predicted_prob_estimate = ((_a * _prob_estimate) * (_a.transpose())) + _q;

		// observation step
		auto innovation = measured_state_matrix - (_h * predicted_state_estimate);
		auto innovation_covariance = (_h * predicted_prob_estimate * (_h.transpose())) + _r;

		// update step
		auto kalman_gain = predicted_prob_estimate * _h.transpose() * (innovation_covariance.inverse());
		_state_estimate = predicted_state_estimate + (kalman_gain * innovation);

		// Eigen::Matrix4d eye;
		// eye.setIdentity();
		// _prob_estimate = (eye - kalman_gain * _h) * predicted_prob_estimate;
		// instead use
		// https://courses.cs.washington.edu/courses/cse466/11au/calendar/14-StateEstimation-posted.pdf
		_prob_estimate = predicted_prob_estimate - (kalman_gain * _h * predicted_prob_estimate);
	}

	Eigen::Matrix<double, N, 1> get_state_estimate()
	{
		return _state_estimate;
	}
};

