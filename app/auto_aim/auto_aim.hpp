/**
 * @file auto_aim.hpp
 * @brief 装甲板自瞄总模块，不包含能量机关。
 */

#ifndef TGU_ROBOCORE_2027_AUTO_AIM_HPP
#define TGU_ROBOCORE_2027_AUTO_AIM_HPP
#pragma once

#include "aimer.hpp"
#include "armor.hpp"
#include "classifier.hpp"
#include "detector.hpp"
#include "solver.hpp"
#include "tracker.hpp"

#include <Eigen/Geometry>

#include <chrono>
#include <list>
#include <optional>
#include <string>

namespace app::auto_aim {

struct AutoAimResult {
    std::list<Armor> armors;
    std::optional<Target> target;
    AimCommand command;
};

class AutoAim {
public:
    explicit AutoAim(const std::string& config_path);

    AutoAimResult process(
        const cv::Mat& image,
        std::chrono::steady_clock::time_point timestamp,
        const Eigen::Matrix3d& gimbal_to_world,
        double bullet_speed);

    [[nodiscard]] const std::string& state() const;

private:
    Detector detector_;
    Classifier classifier_;
    Solver solver_;
    Tracker tracker_;
    Aimer aimer_;
    double camera_delay_ = 0.015;
    std::string state_ = "lost";
};

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_AUTO_AIM_HPP
