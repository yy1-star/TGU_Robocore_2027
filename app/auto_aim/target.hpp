/**
 * @file target.hpp
 * @brief 装甲板目标的 11 维扩展卡尔曼滤波模型。
 */

#ifndef TGU_ROBOCORE_2027_AUTO_AIM_TARGET_HPP
#define TGU_ROBOCORE_2027_AUTO_AIM_TARGET_HPP
#pragma once

#include "armor.hpp"

#include "tools/extended_kalman_filter.hpp"

#include <chrono>
#include <vector>

namespace app::auto_aim {

class Target {
public:
    Target() = default;
    Target(
        const Armor& armor,
        std::chrono::steady_clock::time_point timestamp,
        double radius,
        int armor_count,
        const Eigen::VectorXd& covariance_diagonal);

    void predict(std::chrono::steady_clock::time_point timestamp);
    void predict(double dt);
    void update(const Armor& armor);

    [[nodiscard]] Eigen::VectorXd state() const;
    [[nodiscard]] const tools::ExtendedKalmanFilter& ekf() const;
    [[nodiscard]] std::vector<Eigen::Vector4d> armor_xyza_list() const;
    [[nodiscard]] Eigen::Vector4d predict_armor(double dt, int index) const;
    [[nodiscard]] bool diverged() const;
    [[nodiscard]] bool converged() const;
    [[nodiscard]] bool valid() const;

    ArmorName name = ArmorName::unknown;
    ArmorType type = ArmorType::unknown;
    ArmorPriority priority = ArmorPriority::fifth;
    bool jumped = false;
    int last_id = 0;

private:
    int armor_count_ = 4;
    int switch_count_ = 0;
    int update_count_ = 0;
    bool converged_ = false;
    bool valid_ = false;
    tools::ExtendedKalmanFilter ekf_;
    std::chrono::steady_clock::time_point timestamp_{};

    void update_measurement(const Armor& armor, int id);
    [[nodiscard]] Eigen::Vector3d armor_xyz(const Eigen::VectorXd& state, int id) const;
    [[nodiscard]] Eigen::MatrixXd armor_jacobian(
        const Eigen::VectorXd& state, int id) const;
};

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_AUTO_AIM_TARGET_HPP
