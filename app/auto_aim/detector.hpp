/**
 * @file detector.hpp
 * @brief 基于灯条的装甲板检测。
 */

#ifndef TGU_ROBOCORE_2027_AUTO_AIM_DETECTOR_HPP
#define TGU_ROBOCORE_2027_AUTO_AIM_DETECTOR_HPP
#pragma once

#include "armor.hpp"

#include <list>

namespace app::auto_aim {

struct DetectorConfig {
    ArmorColor enemy_color = ArmorColor::blue;
    double threshold = 150.0;
    double min_lightbar_ratio = 1.2;
    double max_lightbar_ratio = 20.0;
    double min_lightbar_length = 5.0;
    double min_armor_ratio = 1.0;
    double max_armor_ratio = 5.0;
    double max_angle_error = 35.0;
    double max_side_ratio = 2.0;
};

class Detector {
public:
    explicit Detector(DetectorConfig config);

    [[nodiscard]] std::list<Armor> detect(const cv::Mat& image) const;

private:
    DetectorConfig config_;
};

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_AUTO_AIM_DETECTOR_HPP
