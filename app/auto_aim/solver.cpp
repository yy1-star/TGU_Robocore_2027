#include "solver.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include "tools/math_tools.hpp"

namespace app::auto_aim {
namespace {

constexpr double LIGHTBAR_LENGTH = 0.056;
constexpr double SMALL_ARMOR_WIDTH = 0.135;
constexpr double BIG_ARMOR_WIDTH = 0.230;

std::vector<cv::Point3f> object_points(ArmorType type) {
    const auto width = type == ArmorType::big ? BIG_ARMOR_WIDTH : SMALL_ARMOR_WIDTH;
    return {
        {0.0F, static_cast<float>(width / 2.0), static_cast<float>(LIGHTBAR_LENGTH / 2.0)},
        {0.0F, static_cast<float>(-width / 2.0), static_cast<float>(LIGHTBAR_LENGTH / 2.0)},
        {0.0F, static_cast<float>(-width / 2.0), static_cast<float>(-LIGHTBAR_LENGTH / 2.0)},
        {0.0F, static_cast<float>(width / 2.0), static_cast<float>(-LIGHTBAR_LENGTH / 2.0)}};
}

}  // namespace

Solver::Solver(SolverConfig config) : config_(std::move(config)) {}

void Solver::set_gimbal_to_world(const Eigen::Matrix3d& rotation) {
    gimbal_to_world_ = rotation;
}

Eigen::Matrix3d Solver::gimbal_to_world() const {
    return gimbal_to_world_;
}

bool Solver::solve(Armor& armor) const {
    if (config_.camera_matrix.empty() || armor.points.size() != 4) return false;

    cv::Vec3d rvec;
    cv::Vec3d tvec;
    if (!cv::solvePnP(
            object_points(armor.type), armor.points, config_.camera_matrix, config_.distortion,
            rvec, tvec, false, cv::SOLVEPNP_IPPE)) {
        return false;
    }

    const Eigen::Vector3d position_camera{tvec[0], tvec[1], tvec[2]};
    armor.position_gimbal =
        config_.camera_to_gimbal * position_camera + config_.camera_to_gimbal_translation;
    armor.position_world = gimbal_to_world_ * armor.position_gimbal;

    cv::Mat rotation_camera;
    cv::Rodrigues(rvec, rotation_camera);
    Eigen::Matrix3d armor_to_camera;
    cv::cv2eigen(rotation_camera, armor_to_camera);
    const auto armor_to_gimbal = config_.camera_to_gimbal * armor_to_camera;
    const auto armor_to_world = gimbal_to_world_ * armor_to_gimbal;
    armor.ypr_gimbal = tools::eulers(armor_to_gimbal, 2, 1, 0);
    armor.ypr_world = tools::eulers(armor_to_world, 2, 1, 0);
    armor.ypd_world = tools::xyz2ypd(armor.position_world);
    armor.yaw = armor.ypr_world.x();
    armor.yaw_raw = armor.yaw;

    if (config_.optimize_yaw && armor.name != ArmorName::unknown &&
        !((armor.type == ArmorType::big) &&
          (armor.name == ArmorName::three || armor.name == ArmorName::four ||
           armor.name == ArmorName::five))) {
        optimize_yaw(armor);
    }
    return true;
}

std::vector<cv::Point2f> Solver::reproject_armor(
    const Eigen::Vector3d& position_world,
    double yaw,
    ArmorType type,
    ArmorName name) const {
    const auto pitch = name == ArmorName::outpost ? -15.0 : 15.0;
    const auto yaw_rad = yaw;
    const auto pitch_rad = pitch * CV_PI / 180.0;
    const auto sin_yaw = std::sin(yaw_rad);
    const auto cos_yaw = std::cos(yaw_rad);
    const auto sin_pitch = std::sin(pitch_rad);
    const auto cos_pitch = std::cos(pitch_rad);

    Eigen::Matrix3d armor_to_world;
    armor_to_world << cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch,
        sin_yaw * cos_pitch, cos_yaw, sin_yaw * sin_pitch,
        -sin_pitch, 0.0, cos_pitch;

    const auto armor_to_camera =
        config_.camera_to_gimbal.transpose() * gimbal_to_world_.transpose() * armor_to_world;
    const auto armor_position_camera = config_.camera_to_gimbal.transpose() *
                                       (gimbal_to_world_.transpose() * position_world -
                                        config_.camera_to_gimbal_translation);

    cv::Mat rotation_camera;
    cv::eigen2cv(armor_to_camera, rotation_camera);
    cv::Vec3d rvec;
    cv::Rodrigues(rotation_camera, rvec);
    const cv::Vec3d tvec{
        armor_position_camera.x(), armor_position_camera.y(), armor_position_camera.z()};

    std::vector<cv::Point2f> image_points;
    cv::projectPoints(
        object_points(type), rvec, tvec, config_.camera_matrix, config_.distortion, image_points);
    return image_points;
}

double Solver::reprojection_error(const Armor& armor, double yaw, double pitch) const {
    static_cast<void>(pitch);
    const auto image_points =
        reproject_armor(armor.position_world, yaw, armor.type, armor.name);
    if (image_points.size() != armor.points.size()) return 1e10;

    double error = 0.0;
    for (std::size_t index = 0; index < image_points.size(); ++index) {
        error += cv::norm(image_points[index] - armor.points[index]);
    }
    return error;
}

void Solver::optimize_yaw(Armor& armor) const {
    constexpr int SEARCH_RANGE_DEG = 140;
    const auto world_yaw = tools::eulers(gimbal_to_world_, 2, 1, 0).x();
    const auto start = tools::limit_rad(world_yaw - SEARCH_RANGE_DEG * CV_PI / 360.0);

    double min_error = 1e10;
    double best_yaw = armor.yaw;
    for (int degree = 0; degree < SEARCH_RANGE_DEG; ++degree) {
        const auto yaw = tools::limit_rad(start + degree * CV_PI / 180.0);
        const auto error = reprojection_error(armor, yaw, 0.0);
        if (error < min_error) {
            min_error = error;
            best_yaw = yaw;
        }
    }
    armor.yaw_raw = armor.yaw;
    armor.yaw = best_yaw;
    armor.ypr_world.x() = best_yaw;
}

}  // namespace app::auto_aim
