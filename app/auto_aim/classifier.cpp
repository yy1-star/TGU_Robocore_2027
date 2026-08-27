#include "classifier.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <utility>

#include "tools/logger.hpp"

namespace app::auto_aim {
namespace {

constexpr const char* MODULE = "AUTO_AIM_CLASSIFIER";

double armor_width_height_ratio(const Armor& armor) {
    if (armor.points.size() != 4) return 2.0;
    const auto width = cv::norm(armor.points[1] - armor.points[0]);
    const auto height = cv::norm(armor.points[0] - armor.points[3]);
    return height > 1e-6 ? width / height : 2.0;
}

}  // namespace

Classifier::Classifier(const ClassifierConfig& config)
    : min_confidence_(config.min_confidence) {
    if (config.model_path.empty()) return;
    try {
        net_ = cv::dnn::readNetFromONNX(config.model_path);
        LOG_INFO(MODULE, "classifier loaded: {}", config.model_path);
    } catch (const cv::Exception& error) {
        LOG_ERROR(MODULE, "classifier load failed: {}", error.what());
    }
}

void Classifier::classify(std::list<Armor>& armors, const cv::Mat& image) const {
    for (auto& armor : armors) {
        armor.type = armor_type_from_ratio(armor_width_height_ratio(armor));
        armor.name = armor_name_from_class_id(-1);
        armor.priority = ArmorPriority::fifth;

        if (net_.empty() || image.empty() || armor.box.area() <= 0) continue;
        const auto bounds = armor.box & cv::Rect(0, 0, image.cols, image.rows);
        if (bounds.area() <= 0) continue;

        cv::Mat input;
        cv::resize(image(bounds), input, cv::Size(32, 32));
        net_.setInput(cv::dnn::blobFromImage(input, 1.0 / 255.0));
        const auto output = net_.forward().reshape(1, 1);
        cv::Point label;
        double confidence = 0.0;
        cv::minMaxLoc(output, nullptr, &confidence, nullptr, &label);
        armor.class_id = label.x;
        armor.confidence = static_cast<float>(confidence);
        if (confidence < min_confidence_) continue;

        armor.name = armor_name_from_class_id(label.x);
        const auto named_type = armor_type_from_name(armor.name);
        if (named_type != ArmorType::unknown) armor.type = named_type;
        armor.priority = armor_priority(armor.name);
    }
}

}  // namespace app::auto_aim
