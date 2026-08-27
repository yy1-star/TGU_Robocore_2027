#include "detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace app::auto_aim {
namespace {

cv::Point2f midpoint(const cv::Point2f& first, const cv::Point2f& second) {
    return (first + second) * 0.5F;
}

bool nearly_vertical(float angle, double max_angle_error) {
    const auto error = std::abs(std::abs(angle) - CV_PI / 2.0);
    return error <= max_angle_error * CV_PI / 180.0;
}

}  // namespace

Detector::Detector(DetectorConfig config) : config_(std::move(config)) {}

std::list<Armor> Detector::detect(const cv::Mat& image) const {
    std::list<Armor> armors;
    if (image.empty() || image.channels() != 3) return armors;

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::Mat binary;
    cv::threshold(gray, binary, config_.threshold, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    std::vector<Lightbar> lightbars;
    for (const auto& contour : contours) {
        if (contour.size() < 5) continue;

        const auto rotated = cv::minAreaRect(contour);
        const auto width = static_cast<float>(std::max(1.0, std::min(
            static_cast<double>(rotated.size.width), static_cast<double>(rotated.size.height))));
        const auto length = static_cast<float>(std::max(
            static_cast<double>(rotated.size.width), static_cast<double>(rotated.size.height)));
        const auto ratio = length / width;
        if (length < config_.min_lightbar_length || ratio < config_.min_lightbar_ratio ||
            ratio > config_.max_lightbar_ratio) {
            continue;
        }

        std::array<cv::Point2f, 4> corners{};
        rotated.points(corners.data());
        std::sort(corners.begin(), corners.end(), [](const auto& first, const auto& second) {
            return first.y < second.y;
        });
        const auto top = midpoint(corners[0], corners[1]);
        const auto bottom = midpoint(corners[2], corners[3]);
        const auto angle = static_cast<float>(
            std::atan2(bottom.y - top.y, bottom.x - top.x));
        if (!nearly_vertical(angle, config_.max_angle_error)) continue;

        double red_sum = 0.0;
        double blue_sum = 0.0;
        for (const auto& point : contour) {
            const auto& pixel = image.at<cv::Vec3b>(point);
            blue_sum += pixel[0];
            red_sum += pixel[2];
        }
        const auto color = red_sum > blue_sum ? ArmorColor::red : ArmorColor::blue;
        lightbars.push_back({
            rotated.center, top, bottom, length, width, angle, color});
    }

    std::sort(lightbars.begin(), lightbars.end(), [](const auto& first, const auto& second) {
        return first.center.x < second.center.x;
    });

    for (std::size_t first_index = 0; first_index < lightbars.size(); ++first_index) {
        for (std::size_t second_index = first_index + 1;
             second_index < lightbars.size(); ++second_index) {
            const auto& left = lightbars[first_index];
            const auto& right = lightbars[second_index];
            if (left.color != right.color || (config_.enemy_color != ArmorColor::unknown &&
                                              left.color != config_.enemy_color)) {
                continue;
            }

            const auto distance = cv::norm(right.center - left.center);
            const auto ratio = distance / std::max(left.length, right.length);
            const auto side_ratio =
                std::max(left.length, right.length) /
                std::max(1.0F, std::min(left.length, right.length));
            const auto pair_angle = std::abs(std::atan2(
                right.center.y - left.center.y, right.center.x - left.center.x));
            if (ratio < config_.min_armor_ratio || ratio > config_.max_armor_ratio ||
                side_ratio > config_.max_side_ratio ||
                pair_angle > config_.max_angle_error * CV_PI / 180.0) {
                continue;
            }

            Armor armor;
            armor.color = left.color;
            armor.center = midpoint(left.center, right.center);
            armor.points = {left.top, right.top, right.bottom, left.bottom};
            armor.box = cv::boundingRect(armor.points);
            armor.confidence = 1.0F;
            armors.push_back(std::move(armor));
        }
    }
    return armors;
}

}  // namespace app::auto_aim
