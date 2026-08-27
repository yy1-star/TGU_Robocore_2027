/**
 * @file armor.hpp
 * @brief 装甲板和灯条的数据结构。
 */

#ifndef TGU_ROBOCORE_2027_AUTO_AIM_ARMOR_HPP
#define TGU_ROBOCORE_2027_AUTO_AIM_ARMOR_HPP
#pragma once

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <vector>

namespace app::auto_aim {

enum class ArmorColor { red, blue, unknown };
enum class ArmorType { small, big, unknown };
enum class ArmorName {
    one,
    two,
    three,
    four,
    five,
    sentry,
    outpost,
    base,
    unknown
};

enum class ArmorPriority {
    first = 1,
    second = 2,
    third = 3,
    fourth = 4,
    fifth = 5
};

struct Lightbar {
    cv::Point2f center{};
    cv::Point2f top{};
    cv::Point2f bottom{};
    float length = 0.0F;
    float width = 0.0F;
    float angle = 0.0F;
    ArmorColor color = ArmorColor::unknown;
};

struct Armor {
    ArmorColor color = ArmorColor::unknown;
    ArmorType type = ArmorType::unknown;
    ArmorName name = ArmorName::unknown;
    ArmorPriority priority = ArmorPriority::fifth;
    std::vector<cv::Point2f> points;
    cv::Point2f center{};
    cv::Rect box{};
    int class_id = -1;
    float confidence = 0.0F;

    Eigen::Vector3d position_gimbal = Eigen::Vector3d::Zero();
    Eigen::Vector3d position_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d ypr_gimbal = Eigen::Vector3d::Zero();
    Eigen::Vector3d ypr_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d ypd_world = Eigen::Vector3d::Zero();
    double yaw_raw = 0.0;
    double yaw = 0.0;
};

[[nodiscard]] ArmorName armor_name_from_class_id(int class_id);
[[nodiscard]] ArmorType armor_type_from_name(ArmorName name);
[[nodiscard]] ArmorType armor_type_from_ratio(double width_height_ratio);
[[nodiscard]] ArmorPriority armor_priority(ArmorName name);
[[nodiscard]] int armor_count(ArmorName name);
[[nodiscard]] bool is_big_armor(ArmorName name);

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_AUTO_AIM_ARMOR_HPP
