#include "math_tools.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/core.hpp>

namespace tools {

double limit_rad(double angle) {
    while (angle > CV_PI) angle -= 2.0 * CV_PI;
    while (angle <= -CV_PI) angle += 2.0 * CV_PI;
    return angle;
}

Eigen::Vector3d eulers(
    const Eigen::Quaterniond& quaternion, int axis0, int axis1, int axis2, bool extrinsic) {
    if (!extrinsic) std::swap(axis0, axis2);

    const auto i = axis0;
    const auto j = axis1;
    auto k = axis2;
    const bool proper = i == k;
    if (proper) k = 3 - i - j;
    const auto sign = (i - j) * (j - k) * (k - i) / 2;

    const Eigen::Vector4d xyzw = quaternion.coeffs();
    double a;
    double b;
    double c;
    double d;
    if (proper) {
        a = xyzw[3];
        b = xyzw[i];
        c = xyzw[j];
        d = xyzw[k] * sign;
    } else {
        a = xyzw[3] - xyzw[j];
        b = xyzw[i] + xyzw[k] * sign;
        c = xyzw[j] + xyzw[3];
        d = xyzw[k] * sign - xyzw[i];
    }

    Eigen::Vector3d result;
    const double n2 = a * a + b * b + c * c + d * d;
    result[1] = std::acos(std::clamp(2.0 * (a * a + b * b) / n2 - 1.0, -1.0, 1.0));

    const double half_sum = std::atan2(b, a);
    const double half_diff = std::atan2(-d, c);
    const bool safe_first = std::abs(result[1]) >= 1e-7;
    const bool safe_second = std::abs(result[1] - CV_PI) >= 1e-7;
    if (safe_first && safe_second) {
        result[0] = half_sum + half_diff;
        result[2] = half_sum - half_diff;
    } else if (!extrinsic) {
        result[0] = 0.0;
        result[2] = safe_first ? -2.0 * half_diff : 2.0 * half_sum;
    } else {
        result[2] = 0.0;
        result[0] = safe_first ? 2.0 * half_diff : 2.0 * half_sum;
    }

    for (int index = 0; index < 3; ++index) result[index] = limit_rad(result[index]);
    if (!proper) {
        result[2] *= sign;
        result[1] -= CV_PI / 2.0;
    }
    if (!extrinsic) std::swap(result[0], result[2]);
    return result;
}

Eigen::Vector3d eulers(
    const Eigen::Matrix3d& rotation, int axis0, int axis1, int axis2, bool extrinsic) {
    return eulers(Eigen::Quaterniond(rotation), axis0, axis1, axis2, extrinsic);
}

Eigen::Vector3d xyz2ypd(const Eigen::Vector3d& xyz) {
    const double horizontal = std::hypot(xyz.x(), xyz.y());
    return {
        std::atan2(xyz.y(), xyz.x()),
        std::atan2(xyz.z(), horizontal),
        xyz.norm()};
}

Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d& xyz) {
    const double x = xyz.x();
    const double y = xyz.y();
    const double z = xyz.z();
    const double horizontal2 = std::max(1e-12, x * x + y * y);
    const double distance2 = std::max(1e-12, horizontal2 + z * z);
    const double horizontal = std::sqrt(horizontal2);
    Eigen::MatrixXd jacobian(3, 3);
    jacobian << -y / horizontal2, x / horizontal2, 0.0,
        -x * z / (distance2 * horizontal), -y * z / (distance2 * horizontal),
        horizontal / distance2,
        x / std::sqrt(distance2), y / std::sqrt(distance2), z / std::sqrt(distance2);
    return jacobian;
}

double delta_time(
    const std::chrono::steady_clock::time_point& first,
    const std::chrono::steady_clock::time_point& second) {
    return std::chrono::duration<double>(first - second).count();
}

double get_abs_angle(const Eigen::Vector2d& first, const Eigen::Vector2d& second) {
    if (first.norm() == 0.0 || second.norm() == 0.0) return 0.0;
    const double cosine = std::clamp(first.dot(second) / first.norm() / second.norm(), -1.0, 1.0);
    return std::acos(cosine);
}

}  // namespace tools
