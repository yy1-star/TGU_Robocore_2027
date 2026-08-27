/**
 * @file tracker.hpp
 * @brief 装甲板目标选择、状态机和 EKF 跟踪。
 */

#ifndef TGU_ROBOCORE_2027_AUTO_AIM_TRACKER_HPP
#define TGU_ROBOCORE_2027_AUTO_AIM_TRACKER_HPP
#pragma once

#include "armor.hpp"
#include "target.hpp"

#include <chrono>
#include <list>
#include <optional>
#include <string>

namespace app::auto_aim {

struct TrackerConfig {
    int min_detect_count = 2;
    int max_temp_lost_count = 10;
    int outpost_max_temp_lost_count = 30;
    double normal_radius = 0.2;
    double outpost_radius = 0.2765;
    double base_radius = 0.3205;
};

class Tracker {
public:
    explicit Tracker(TrackerConfig config);

    [[nodiscard]] std::string state() const;
    std::optional<Target> track(
        std::list<Armor>& armors, std::chrono::steady_clock::time_point timestamp);

private:
    TrackerConfig config_;
    std::string state_ = "lost";
    int detect_count_ = 0;
    int temp_lost_count_ = 0;
    std::chrono::steady_clock::time_point last_timestamp_{};
    std::optional<Target> target_;

    [[nodiscard]] std::list<Armor> sorted_candidates(const std::list<Armor>& armors) const;
    void set_target(const Armor& armor, std::chrono::steady_clock::time_point timestamp);
    bool update_target(
        const std::list<Armor>& armors, std::chrono::steady_clock::time_point timestamp);
};

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_AUTO_AIM_TRACKER_HPP
