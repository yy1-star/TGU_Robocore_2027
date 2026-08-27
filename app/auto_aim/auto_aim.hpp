/**
 * @file auto_aim.hpp
 * @brief 装甲板自瞄应用模块，不包含能量机关。
 */

#ifndef TGU_ROBOCORE_2027_AUTO_AIM_HPP
#define TGU_ROBOCORE_2027_AUTO_AIM_HPP

#pragma once

#include <Eigen/Geometry>
#include <chrono>
#include <list>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <vector>

namespace app::auto_aim {

enum class ArmorColor { red, blue, unknown };
enum class ArmorType { small, big, unknown };
enum class ArmorName { one, two, three, four, five, sentry, outpost, base, unknown };

struct Armor {
    ArmorColor color = ArmorColor::unknown;
    ArmorType type = ArmorType::unknown;
    ArmorName name = ArmorName::unknown;
    std::vector<cv::Point2f> points;
    cv::Point2f center{};
    float confidence = 0.0F;
    double yaw = 0.0;
    Eigen::Vector3d position_gimbal = Eigen::Vector3d::Zero();
    Eigen::Vector3d position_world = Eigen::Vector3d::Zero();
};

struct Target {
    ArmorName name = ArmorName::unknown;
    ArmorType type = ArmorType::unknown;
    Eigen::Vector3d center_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_world = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double yaw_velocity = 0.0;
    double radius = 0.2;
    int armor_count = 4;
    bool valid = false;

    [[nodiscard]] Eigen::Vector4d predict_armor(double dt, int index) const;
};

struct AimCommand {
    bool valid = false;
    bool fire = false;
    double yaw = 0.0;
    double pitch = 0.0;
    double flight_time = 0.0;
};

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
    struct Lightbar {
        cv::Point2f center{};
        cv::Point2f top{};
        cv::Point2f bottom{};
        float length = 0.0F;
        float width = 0.0F;
        ArmorColor color = ArmorColor::unknown;
    };

    struct Config {
        ArmorColor enemy_color = ArmorColor::blue;
        double threshold = 150.0;
        double min_lightbar_ratio = 1.2;
        double max_lightbar_ratio = 20.0;
        double min_lightbar_length = 5.0;
        double min_armor_ratio = 1.0;
        double max_armor_ratio = 5.0;
        double max_angle_error = 35.0;
        double max_side_ratio = 2.0;
        double min_classifier_confidence = 0.4;
        double yaw_offset = 0.0;
        double pitch_offset = 0.0;
        double camera_delay = 0.015;
        double gravity = 9.81;
        std::string classifier_model;
        cv::Mat camera_matrix;
        cv::Mat distortion;
    };

    Config config_;
    cv::dnn::Net classifier_;
    Target target_;
    std::string state_ = "lost";
    std::chrono::steady_clock::time_point last_timestamp_{};
    Eigen::Matrix3d gimbal_to_world_ = Eigen::Matrix3d::Identity();

    static Config load_config(const std::string& config_path);
    std::list<Armor> detect_armors(const cv::Mat& image) const;
    std::optional<Armor> classify_and_select(std::list<Armor>& armors, const cv::Mat& image);
    void solve_pose(Armor& armor) const;
    void update_target(const Armor& armor, std::chrono::steady_clock::time_point timestamp);
    AimCommand aim(const Target& target, double bullet_speed, double now_delay) const;
    static double wrap_angle(double angle);
    static ArmorColor parse_color(const std::string& value);
    static ArmorName parse_name(int class_id);
    static ArmorType parse_type(ArmorName name, double ratio);
};

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_AUTO_AIM_HPP
