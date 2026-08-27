#include "aimer.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace app::auto_aim {

Aimer::Aimer(AimerConfig config) : config_(std::move(config)) {}

AimCommand Aimer::aim(
    const Target& target, double bullet_speed, double processing_delay) const {
    AimCommand command;
    if (!target.valid()) return command;
    if (bullet_speed < 14.0 || bullet_speed > 35.0) bullet_speed = 23.0;

    const auto delay = target.state().size() > 7 && std::abs(target.state()[7]) > config_.decision_speed
                           ? config_.high_speed_delay
                           : config_.low_speed_delay;
    auto flight_time = 0.0;
    Eigen::Vector4d aim_point = choose_aim_point(target, processing_delay + delay);

    tools::Trajectory trajectory(
        bullet_speed, aim_point.head<2>().norm(), aim_point.z(), config_.gravity);
    if (trajectory.unsolvable) return command;

    for (int iteration = 0; iteration < 10; ++iteration) {
        aim_point = choose_aim_point(target, processing_delay + delay + flight_time);
        trajectory = tools::Trajectory(
            bullet_speed, aim_point.head<2>().norm(), aim_point.z(), config_.gravity);
        if (trajectory.unsolvable) return command;
        if (std::abs(trajectory.fly_time - flight_time) < 0.001) break;
        flight_time = trajectory.fly_time;
    }

    command.valid = std::isfinite(trajectory.pitch) && std::isfinite(aim_point.x());
    if (!command.valid) return command;
    command.yaw = tools::limit_rad(
        std::atan2(aim_point.y(), aim_point.x()) + config_.yaw_offset);
    command.pitch = -(trajectory.pitch + config_.pitch_offset);
    command.flight_time = trajectory.fly_time;
    command.fire = std::abs(target.state()[7]) < 20.0;
    return command;
}

Eigen::Vector4d Aimer::choose_aim_point(const Target& target, double dt) const {
    const auto armors = target.armor_xyza_list();
    if (armors.empty()) return Eigen::Vector4d::Zero();
    if (!target.jumped || target.state().size() <= 7) return target.predict_armor(dt, 0);

    const auto state = target.state();
    const auto center_yaw = std::atan2(state[2], state[0]);
    std::vector<int> available;
    for (std::size_t index = 0; index < armors.size(); ++index) {
        const auto delta = tools::limit_rad(armors[index].w() - center_yaw);
        if (std::abs(delta) <= 60.0 * CV_PI / 180.0) {
            available.push_back(static_cast<int>(index));
        }
    }
    if (available.empty()) return target.predict_armor(dt, 0);
    if (available.size() == 1) {
        lock_id_ = -1;
        return target.predict_armor(dt, available.front());
    }

    if (std::find(available.begin(), available.end(), lock_id_) == available.end()) {
        lock_id_ = available.front();
        auto best_delta = std::abs(tools::limit_rad(
            armors[lock_id_].w() - center_yaw));
        for (const auto index : available) {
            const auto delta = std::abs(tools::limit_rad(armors[index].w() - center_yaw));
            if (delta < best_delta) {
                best_delta = delta;
                lock_id_ = index;
            }
        }
    }
    return target.predict_armor(dt, lock_id_);
}

}  // namespace app::auto_aim
