#include "extended_kalman_filter.hpp"

#include <numeric>
#include <utility>

namespace tools {

ExtendedKalmanFilter::ExtendedKalmanFilter(
    const Eigen::VectorXd& initial_state,
    const Eigen::MatrixXd& initial_covariance,
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> x_add)
    : x(initial_state),
      P(initial_covariance),
      identity_(Eigen::MatrixXd::Identity(initial_state.size(), initial_state.size())),
      x_add_(std::move(x_add)) {}

void ExtendedKalmanFilter::predict(
    const Eigen::MatrixXd& transition,
    const Eigen::MatrixXd& process_noise,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& state_function) {
    P = transition * P * transition.transpose() + process_noise;
    x = state_function(x);
}

void ExtendedKalmanFilter::update(
    const Eigen::VectorXd& measurement,
    const Eigen::MatrixXd& jacobian,
    const Eigen::MatrixXd& measurement_noise,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& measurement_function,
    const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>& subtract) {
    const Eigen::MatrixXd innovation_covariance =
        jacobian * P * jacobian.transpose() + measurement_noise;
    const Eigen::MatrixXd gain =
        P * jacobian.transpose() * innovation_covariance.ldlt().solve(
                                        Eigen::MatrixXd::Identity(measurement.size(), measurement.size()));

    const Eigen::VectorXd innovation = subtract(measurement, measurement_function(x));
    P = (identity_ - gain * jacobian) * P *
            (identity_ - gain * jacobian).transpose() +
        gain * measurement_noise * gain.transpose();
    x = x_add_(x, gain * innovation);

    const Eigen::VectorXd residual = subtract(measurement, measurement_function(x));
    last_nis = (residual.transpose() *
                innovation_covariance.ldlt().solve(residual))(0, 0);
    recent_nis_failures.push_back(last_nis > 9.49 ? 1 : 0);
    if (recent_nis_failures.size() > window_size) recent_nis_failures.pop_front();
}

}  // namespace tools
