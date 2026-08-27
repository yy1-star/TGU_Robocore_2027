/**
 * @file aimer.hpp
 * @brief 基于目标预测和弹道解算的瞄准输出。
 */

#ifndef TGU_ROBOCORE_2027_AUTO_AIM_AIMER_HPP
#define TGU_ROBOCORE_2027_AUTO_AIM_AIMER_HPP
#pragma once

#include "target.hpp"

#include <opencv2/core.hpp>

namespace app::auto_aim {

struct AimerConfig {
    double yaw_offset = 0.0;
    double pitch_offset = 0.0;
    double gravity = 9.81;
    double coming_angle = 70.0 * CV_PI / 180.0;
    double leaving_angle = 30.0 * CV_PI / 180.0;
    double camera_delay = 0.015;
    double high_speed_delay = 0.1;
    double low_speed_delay = 0.1;
    double decision_speed = 2.0;
};

struct AimCommand {
    bool valid = false;
    bool fire = false;
    double yaw = 0.0;
    double pitch = 0.0;
    double flight_time = 0.0;
};

class Aimer {
public:
    explicit Aimer(AimerConfig config);

    [[nodiscard]] AimCommand aim(
        const Target& target, double bullet_speed, double processing_delay) const;

private:
    AimerConfig config_;
    mutable int lock_id_ = -1;

    [[nodiscard]] Eigen::Vector4d choose_aim_point(const Target& target, double dt) const;
};

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_AUTO_AIM_AIMER_HPP
