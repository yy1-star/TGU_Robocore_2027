#ifndef TGU_ROBOCORE_2027_TOOLS_TRAJECTORY_HPP
#define TGU_ROBOCORE_2027_TOOLS_TRAJECTORY_HPP

#pragma once

namespace tools {

struct Trajectory {
    bool unsolvable = true;
    double fly_time = 0.0;
    double pitch = 0.0;

    Trajectory(
        double bullet_speed,
        double horizontal_distance,
        double height,
        double gravity = 9.7833);
};

}  // namespace tools

#endif  // TGU_ROBOCORE_2027_TOOLS_TRAJECTORY_HPP
