/**
 * @file solver.hpp
 * @brief 装甲板 PnP、坐标变换和 yaw 优化。
 */

#ifndef TGU_ROBOCORE_2027_AUTO_AIM_SOLVER_HPP
#define TGU_ROBOCORE_2027_AUTO_AIM_SOLVER_HPP
#pragma once

#include "armor.hpp"

#include <Eigen/Geometry>

#include <opencv2/core.hpp>

#include <vector>

namespace app::auto_aim {

struct SolverConfig {
    cv::Mat camera_matrix;
    cv::Mat distortion;
    // OpenCV camera: x-right, y-down, z-forward.
    // Gimbal: x-forward, y-right, z-up.
    Eigen::Matrix3d camera_to_gimbal{
        {0.0, 0.0, 1.0},
        {1.0, 0.0, 0.0},
        {0.0, -1.0, 0.0}};
    Eigen::Vector3d camera_to_gimbal_translation = Eigen::Vector3d::Zero();
    bool optimize_yaw = true;
};

class Solver {
public:
    explicit Solver(SolverConfig config);

    void set_gimbal_to_world(const Eigen::Matrix3d& rotation);
    [[nodiscard]] Eigen::Matrix3d gimbal_to_world() const;
    bool solve(Armor& armor) const;

private:
    SolverConfig config_;
    Eigen::Matrix3d gimbal_to_world_ = Eigen::Matrix3d::Identity();

    [[nodiscard]] std::vector<cv::Point2f> reproject_armor(
        const Eigen::Vector3d& position_world,
        double yaw,
        ArmorType type,
        ArmorName name) const;
    [[nodiscard]] double reprojection_error(
        const Armor& armor, double yaw, double pitch) const;
    void optimize_yaw(Armor& armor) const;
};

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_AUTO_AIM_SOLVER_HPP
