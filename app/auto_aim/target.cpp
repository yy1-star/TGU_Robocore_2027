#include "target.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

#include "tools/math_tools.hpp"

namespace app::auto_aim {

Target::Target(
    const Armor& armor,
    std::chrono::steady_clock::time_point timestamp,
    double radius,
    int armor_count,
    const Eigen::VectorXd& covariance_diagonal)
    : name(armor.name),
      type(armor.type),
      priority(armor.priority),
      armor_count_(std::max(1, armor_count)),
      timestamp_(timestamp),
      valid_(true) {
    const auto center_yaw = armor.yaw;
    const Eigen::VectorXd initial_state{
        armor.position_world.x() + radius * std::cos(center_yaw),
        0.0,
        armor.position_world.y() + radius * std::sin(center_yaw),
        0.0,
        armor.position_world.z(),
        0.0,
        center_yaw,
        0.0,
        radius,
        0.0,
        0.0};

    auto x_add = [](const Eigen::VectorXd& first, const Eigen::VectorXd& second) {
        auto result = first + second;
        result[6] = tools::limit_rad(result[6]);
        return result;
    };
    ekf_ = tools::ExtendedKalmanFilter(
        initial_state, covariance_diagonal.asDiagonal(), std::move(x_add));
}

void Target::predict(std::chrono::steady_clock::time_point timestamp) {
    const auto dt = tools::delta_time(timestamp, timestamp_);
    predict(dt);
    timestamp_ = timestamp;
}

void Target::predict(double dt) {
    if (!valid_ || dt <= 0.0) return;
    dt = std::min(dt, 0.2);

    Eigen::MatrixXd transition = Eigen::MatrixXd::Identity(11, 11);
    transition(0, 1) = dt;
    transition(2, 3) = dt;
    transition(4, 5) = dt;
    transition(6, 7) = dt;

    const auto position_variance = name == ArmorName::outpost ? 10.0 : 100.0;
    const auto angular_variance = name == ArmorName::outpost ? 0.1 : 400.0;
    const auto a = dt * dt * dt * dt / 4.0;
    const auto b = dt * dt * dt / 2.0;
    const auto c = dt * dt;
    Eigen::MatrixXd process_noise = Eigen::MatrixXd::Zero(11, 11);
    for (const auto offset : {0, 2, 4}) {
        process_noise(offset, offset) = a * position_variance;
        process_noise(offset, offset + 1) = b * position_variance;
        process_noise(offset + 1, offset) = b * position_variance;
        process_noise(offset + 1, offset + 1) = c * position_variance;
    }
    process_noise(6, 6) = a * angular_variance;
    process_noise(6, 7) = b * angular_variance;
    process_noise(7, 6) = b * angular_variance;
    process_noise(7, 7) = c * angular_variance;

    const auto state_function = [transition](const Eigen::VectorXd& state) {
        auto result = transition * state;
        result[6] = tools::limit_rad(result[6]);
        return result;
    };
    ekf_.predict(transition, process_noise, state_function);

    if (converged() && name == ArmorName::outpost && std::abs(ekf_.x[7]) > 2.0) {
        ekf_.x[7] = ekf_.x[7] > 0.0 ? 2.51 : -2.51;
    }
}

void Target::update(const Armor& armor) {
    if (!valid_) return;

    const auto candidates = armor_xyza_list();
    std::vector<std::pair<double, int>> ranked;
    ranked.reserve(candidates.size());
    for (std::size_t index = 0; index < candidates.size(); ++index) {
        const auto ypd = tools::xyz2ypd(candidates[index].head<3>());
        const auto angle_error =
            std::abs(tools::limit_rad(armor.yaw - candidates[index].w())) +
            std::abs(tools::limit_rad(armor.ypd_world.x() - ypd.x()));
        ranked.emplace_back(angle_error, static_cast<int>(index));
    }
    std::sort(ranked.begin(), ranked.end());
    const auto id = ranked.empty() ? 0 : ranked.front().second;

    jumped = id != 0;
    if (id != last_id) ++switch_count_;
    last_id = id;
    ++update_count_;
    update_measurement(armor, id);
    if (update_count_ > (name == ArmorName::outpost ? 10 : 3) && !diverged()) {
        converged_ = true;
    }
}

Eigen::VectorXd Target::state() const {
    return ekf_.x;
}

const tools::ExtendedKalmanFilter& Target::ekf() const {
    return ekf_;
}

std::vector<Eigen::Vector4d> Target::armor_xyza_list() const {
    std::vector<Eigen::Vector4d> result;
    if (!valid_) return result;
    result.reserve(armor_count_);
    for (int index = 0; index < armor_count_; ++index) {
        const auto angle = tools::limit_rad(
            ekf_.x[6] + index * 2.0 * CV_PI / static_cast<double>(armor_count_));
        const auto xyz = armor_xyz(ekf_.x, index);
        result.push_back({xyz.x(), xyz.y(), xyz.z(), angle});
    }
    return result;
}

Eigen::Vector4d Target::predict_armor(double dt, int index) const {
    if (!valid_) return Eigen::Vector4d::Zero();
    auto state = ekf_.x;
    state[0] += state[1] * dt;
    state[2] += state[3] * dt;
    state[4] += state[5] * dt;
    state[6] = tools::limit_rad(state[6] + state[7] * dt);
    const auto angle = tools::limit_rad(
        state[6] + index * 2.0 * CV_PI / static_cast<double>(armor_count_));
    const auto xyz = armor_xyz(state, index);
    return {xyz.x(), xyz.y(), xyz.z(), angle};
}

bool Target::diverged() const {
    if (!valid_ || ekf_.x.size() < 11) return true;
    const auto radius_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
    const auto long_radius = ekf_.x[8] + ekf_.x[9];
    const auto long_radius_ok = long_radius > 0.05 && long_radius < 0.5;
    return !(radius_ok && long_radius_ok);
}

bool Target::converged() const {
    return converged_;
}

bool Target::valid() const {
    return valid_ && !diverged();
}

void Target::update_measurement(const Armor& armor, int id) {
    const auto jacobian = armor_jacobian(ekf_.x, id);
    const auto center_yaw = std::atan2(armor.position_world.y(), armor.position_world.x());
    const auto delta_angle = tools::limit_rad(armor.yaw - center_yaw);
    const Eigen::VectorXd measurement{
        armor.ypd_world.x(), armor.ypd_world.y(), armor.ypd_world.z(), armor.yaw};
    const Eigen::VectorXd noise_diagonal{
        4e-3, 4e-3, std::log(std::abs(delta_angle) + 1.0) + 1.0,
        std::log(std::abs(armor.ypd_world.z()) + 1.0) / 200.0 + 9e-2};
    const auto measurement_noise = noise_diagonal.asDiagonal();

    const auto measurement_function = [this, id](const Eigen::VectorXd& state) {
        const auto ypd = tools::xyz2ypd(armor_xyz(state, id));
        return Eigen::Vector4d{
            ypd.x(), ypd.y(), ypd.z(),
            tools::limit_rad(state[6] + id * 2.0 * CV_PI / armor_count_)};
    };
    const auto subtract = [](const Eigen::VectorXd& first, const Eigen::VectorXd& second) {
        auto result = first - second;
        result[0] = tools::limit_rad(result[0]);
        result[1] = tools::limit_rad(result[1]);
        result[3] = tools::limit_rad(result[3]);
        return result;
    };
    ekf_.update(measurement, jacobian, measurement_noise, measurement_function, subtract);
}

Eigen::Vector3d Target::armor_xyz(const Eigen::VectorXd& state, int id) const {
    const auto angle =
        tools::limit_rad(state[6] + id * 2.0 * CV_PI / static_cast<double>(armor_count_));
    const auto use_long_radius = armor_count_ == 4 && (id == 1 || id == 3);
    const auto radius = use_long_radius ? state[8] + state[9] : state[8];
    const auto height = use_long_radius ? state[4] + state[10] : state[4];
    return {
        state[0] - radius * std::cos(angle),
        state[2] - radius * std::sin(angle),
        height};
}

Eigen::MatrixXd Target::armor_jacobian(const Eigen::VectorXd& state, int id) const {
    const auto angle =
        tools::limit_rad(state[6] + id * 2.0 * CV_PI / static_cast<double>(armor_count_));
    const auto use_long_radius = armor_count_ == 4 && (id == 1 || id == 3);
    const auto radius = use_long_radius ? state[8] + state[9] : state[8];
    const auto dx_da = radius * std::sin(angle);
    const auto dy_da = -radius * std::cos(angle);
    const auto dx_dr = -std::cos(angle);
    const auto dy_dr = -std::sin(angle);
    const auto dx_dl = use_long_radius ? dx_dr : 0.0;
    const auto dy_dl = use_long_radius ? dy_dr : 0.0;
    const auto dz_dh = use_long_radius ? 1.0 : 0.0;

    Eigen::MatrixXd armor_xyz_jacobian(4, 11);
    armor_xyz_jacobian << 1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl, 0,
        0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dz_dh,
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;

    const auto xyz_jacobian = tools::xyz2ypd_jacobian(armor_xyz(state, id));
    Eigen::MatrixXd ypd_jacobian = Eigen::MatrixXd::Zero(4, 4);
    ypd_jacobian.block<3, 3>(0, 0) = xyz_jacobian;
    ypd_jacobian(3, 3) = 1.0;
    return ypd_jacobian * armor_xyz_jacobian;
}

}  // namespace app::auto_aim
