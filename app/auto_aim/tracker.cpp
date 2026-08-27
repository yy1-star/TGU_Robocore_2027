#include "tracker.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

#include "tools/math_tools.hpp"

namespace app::auto_aim {

Tracker::Tracker(TrackerConfig config) : config_(std::move(config)) {}

std::string Tracker::state() const {
    return state_;
}

std::optional<Target> Tracker::track(
    std::list<Armor>& armors, std::chrono::steady_clock::time_point timestamp) {
    const auto candidates = sorted_candidates(armors);
    const auto dt = last_timestamp_.time_since_epoch().count() == 0
                        ? 0.0
                        : std::chrono::duration<double>(timestamp - last_timestamp_).count();
    last_timestamp_ = timestamp;

    if (dt > 0.1 && state_ != "lost") {
        state_ = "lost";
        target_.reset();
    }

    if (state_ == "lost") {
        if (!candidates.empty()) {
            set_target(candidates.front(), timestamp);
            state_ = "detecting";
            detect_count_ = 1;
        }
    } else if (state_ == "detecting") {
        if (!candidates.empty() && target_.has_value() &&
            candidates.front().name == target_->name &&
            candidates.front().type == target_->type) {
            target_->update(candidates.front());
            ++detect_count_;
            if (detect_count_ >= config_.min_detect_count) state_ = "tracking";
        } else {
            state_ = "lost";
            target_.reset();
            detect_count_ = 0;
        }
    } else if (!candidates.empty() && target_.has_value() &&
               candidates.front().priority < target_->priority) {
        set_target(candidates.front(), timestamp);
        state_ = "detecting";
        detect_count_ = 1;
        temp_lost_count_ = 0;
    } else {
        const auto found = update_target(candidates, timestamp);
        if (found) {
            state_ = "tracking";
            temp_lost_count_ = 0;
        } else {
            state_ = "temp_lost";
            ++temp_lost_count_;
            const auto max_lost = target_.has_value() &&
                                          target_->name == ArmorName::outpost
                                      ? config_.outpost_max_temp_lost_count
                                      : config_.max_temp_lost_count;
            if (temp_lost_count_ > max_lost) {
                state_ = "lost";
                target_.reset();
                temp_lost_count_ = 0;
            }
        }
    }

    if (target_.has_value() && target_->diverged()) {
        state_ = "lost";
        target_.reset();
    }
    if (target_.has_value() &&
        std::accumulate(
            target_->ekf().recent_nis_failures.begin(),
            target_->ekf().recent_nis_failures.end(),
            0) >= static_cast<int>(0.4 * target_->ekf().window_size)) {
        state_ = "lost";
        target_.reset();
    }
    if (state_ == "lost") return std::nullopt;
    return target_;
}

std::list<Armor> Tracker::sorted_candidates(const std::list<Armor>& armors) const {
    std::list<Armor> candidates;
    for (const auto& armor : armors) {
        if (armor.color == ArmorColor::unknown) continue;
        candidates.push_back(armor);
    }

    candidates.sort([](const Armor& first, const Armor& second) {
        if (first.priority != second.priority) return first.priority < second.priority;
        return cv::norm(first.center) < cv::norm(second.center);
    });
    return candidates;
}

void Tracker::set_target(
    const Armor& armor, std::chrono::steady_clock::time_point timestamp) {
    double radius = config_.normal_radius;
    if (armor.name == ArmorName::outpost) radius = config_.outpost_radius;
    if (armor.name == ArmorName::base) radius = config_.base_radius;

    Eigen::VectorXd covariance(11);
    covariance << 1.0, 64.0, 1.0, 64.0, 1.0, 64.0, 0.4, 100.0, 1.0, 1.0, 1.0;
    if (armor.name == ArmorName::outpost || armor.name == ArmorName::base) {
        covariance[8] = 1e-4;
        covariance[9] = 0.0;
        covariance[10] = 0.0;
    }
    target_.emplace(armor, timestamp, radius, armor_count(armor.name), covariance);
}

bool Tracker::update_target(
    const std::list<Armor>& armors, std::chrono::steady_clock::time_point timestamp) {
    if (!target_.has_value()) return false;
    target_->predict(timestamp);

    auto best = armors.end();
    auto best_error = 1e10;
    for (auto iterator = armors.begin(); iterator != armors.end(); ++iterator) {
        if (iterator->name != target_->name || iterator->type != target_->type) continue;
        const auto predicted = target_->armor_xyza_list();
        if (predicted.empty()) continue;
        for (const auto& prediction : predicted) {
            const auto predicted_ypd = tools::xyz2ypd(prediction.head<3>());
            const auto error =
                std::abs(tools::limit_rad(iterator->yaw - prediction.w())) +
                std::abs(tools::limit_rad(iterator->ypd_world.x() - predicted_ypd.x()));
            if (error < best_error) {
                best_error = error;
                best = iterator;
            }
        }
    }
    if (best == armors.end()) return false;
    target_->update(*best);
    return true;
}

}  // namespace app::auto_aim
