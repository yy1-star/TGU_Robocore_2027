#ifndef TGU_ROBOCORE_2027_TOOLS_EXTENDED_KALMAN_FILTER_HPP
#define TGU_ROBOCORE_2027_TOOLS_EXTENDED_KALMAN_FILTER_HPP

#pragma once

#include <Eigen/Dense>
#include <deque>
#include <functional>

namespace tools {

class ExtendedKalmanFilter {
public:
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
    std::deque<int> recent_nis_failures{0};
    std::size_t window_size = 100;
    double last_nis = 0.0;

    ExtendedKalmanFilter() = default;
    ExtendedKalmanFilter(
        const Eigen::VectorXd& initial_state,
        const Eigen::MatrixXd& initial_covariance,
        std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> x_add);

    void predict(
        const Eigen::MatrixXd& transition,
        const Eigen::MatrixXd& process_noise,
        const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& state_function);

    void update(
        const Eigen::VectorXd& measurement,
        const Eigen::MatrixXd& jacobian,
        const Eigen::MatrixXd& measurement_noise,
        const std::function<Eigen::VectorXd(const Eigen::VectorXd&)>& measurement_function,
        const std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)>&
            subtract);

private:
    Eigen::MatrixXd identity_;
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> x_add_;
};

}  // namespace tools

#endif  // TGU_ROBOCORE_2027_TOOLS_EXTENDED_KALMAN_FILTER_HPP
