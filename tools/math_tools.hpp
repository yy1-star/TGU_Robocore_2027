#ifndef TGU_ROBOCORE_2027_TOOLS_MATH_TOOLS_HPP
#define TGU_ROBOCORE_2027_TOOLS_MATH_TOOLS_HPP

#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>

namespace tools {

double limit_rad(double angle);
Eigen::Vector3d eulers(
    const Eigen::Quaterniond& quaternion, int axis0, int axis1, int axis2,
    bool extrinsic = false);
Eigen::Vector3d eulers(
    const Eigen::Matrix3d& rotation, int axis0, int axis1, int axis2,
    bool extrinsic = false);
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d& xyz);
Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d& xyz);
double delta_time(
    const std::chrono::steady_clock::time_point& first,
    const std::chrono::steady_clock::time_point& second);
double get_abs_angle(const Eigen::Vector2d& first, const Eigen::Vector2d& second);

template <typename T>
constexpr T square(const T& value) {
    return value * value;
}

}  // namespace tools

#endif  // TGU_ROBOCORE_2027_TOOLS_MATH_TOOLS_HPP
