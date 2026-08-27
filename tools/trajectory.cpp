#include "trajectory.hpp"

#include <cmath>

namespace tools {

Trajectory::Trajectory(
    double bullet_speed, double horizontal_distance, double height, double gravity) {
    if (bullet_speed <= 0.0 || horizontal_distance <= 0.0 || gravity <= 0.0) return;
    const double a = gravity * horizontal_distance * horizontal_distance /
                     (2.0 * bullet_speed * bullet_speed);
    const double b = -horizontal_distance;
    const double c = a + height;
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0 || a == 0.0) return;

    const double root = std::sqrt(discriminant);
    const double pitch_a = std::atan((-b + root) / (2.0 * a));
    const double pitch_b = std::atan((-b - root) / (2.0 * a));
    const double time_a = horizontal_distance / (bullet_speed * std::cos(pitch_a));
    const double time_b = horizontal_distance / (bullet_speed * std::cos(pitch_b));

    unsolvable = false;
    if (time_a < time_b) {
        pitch = pitch_a;
        fly_time = time_a;
    } else {
        pitch = pitch_b;
        fly_time = time_b;
    }
}

}  // namespace tools
