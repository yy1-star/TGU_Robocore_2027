/**
 * @file classifier.hpp
 * @brief 装甲板数字分类和类型判断。
 */

#ifndef TGU_ROBOCORE_2027_AUTO_AIM_CLASSIFIER_HPP
#define TGU_ROBOCORE_2027_AUTO_AIM_CLASSIFIER_HPP
#pragma once

#include "armor.hpp"

#include <opencv2/dnn.hpp>

#include <list>
#include <string>

namespace app::auto_aim {

struct ClassifierConfig {
    std::string model_path;
    double min_confidence = 0.4;
};

class Classifier {
public:
    explicit Classifier(const ClassifierConfig& config);

    void classify(std::list<Armor>& armors, const cv::Mat& image) const;

private:
    mutable cv::dnn::Net net_;
    double min_confidence_;
};

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_AUTO_AIM_CLASSIFIER_HPP
