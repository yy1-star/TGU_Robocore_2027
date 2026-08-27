#include "auto_aim.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include "tools/logger.hpp"
#include "tools/tomlpp.hpp"

namespace app::auto_aim {
namespace {

constexpr const char* MODULE = "AUTO_AIM";
constexpr double LIGHTBAR_LENGTH = 0.056;
constexpr double SMALL_ARMOR_WIDTH = 0.135;
constexpr double BIG_ARMOR_WIDTH = 0.230;

template <typename Node>
double read_number(const Node& node, const char* key, double fallback) {
    return node[key].value_or(fallback);
}

cv::Point2f midpoint(const cv::Point2f& a, const cv::Point2f& b) {
    return (a + b) * 0.5F;
}

}  // namespace

Eigen::Vector4d Target::predict_armor(double dt, int index) const {
    const double angle = yaw + yaw_velocity * dt +
                         static_cast<double>(index) * 2.0 * CV_PI / armor_count;
    const Eigen::Vector3d center = center_world + velocity_world * dt;
    return {
        center.x() - radius * std::cos(angle),
        center.y() - radius * std::sin(angle),
        center.z(),
        angle};
}

AutoAim::AutoAim(const std::string& config_path) : config_(load_config(config_path)) {
    if (!config_.classifier_model.empty()) {
        try {
            classifier_ = cv::dnn::readNetFromONNX(config_.classifier_model);
            LOG_INFO(MODULE, "classifier loaded: {}", config_.classifier_model);
        } catch (const cv::Exception& error) {
            LOG_ERROR(MODULE, "classifier load failed: {}", error.what());
        }
    }
}

AutoAimResult AutoAim::process(
    const cv::Mat& image,
    std::chrono::steady_clock::time_point timestamp,
    const Eigen::Matrix3d& gimbal_to_world,
    double bullet_speed) {
    AutoAimResult result;
    if (image.empty()) {
        state_ = "lost";
        return result;
    }

    gimbal_to_world_ = gimbal_to_world;
    auto armors = detect_armors(image);
    for (auto& armor : armors) {
        solve_pose(armor);
    }
    result.armors = armors;

    const auto selected = classify_and_select(armors, image);
    const double elapsed = last_timestamp_.time_since_epoch().count() == 0
                               ? 0.0
                               : std::chrono::duration<double>(timestamp - last_timestamp_).count();

    if (!selected.has_value()) {
        if (target_.valid && elapsed < 0.1) {
            target_.center_world += target_.velocity_world * elapsed;
        } else {
            target_.valid = false;
        }
        state_ = target_.valid ? "temp_lost" : "lost";
        last_timestamp_ = timestamp;
        return result;
    }

    update_target(*selected, timestamp);
    last_timestamp_ = timestamp;
    state_ = "tracking";
    result.target = target_;
    result.command = aim(target_, bullet_speed, config_.camera_delay);
    return result;
}

const std::string& AutoAim::state() const {
    return state_;
}

AutoAim::Config AutoAim::load_config(const std::string& config_path) {
    Config config;
    const auto table = toml::parse_file(config_path);
    const auto node = table["auto_aim"];

    config.enemy_color = parse_color(node["enemy_color"].value_or(std::string("blue")));
    config.threshold = read_number(node, "threshold", config.threshold);
    config.min_lightbar_ratio = read_number(node, "min_lightbar_ratio", config.min_lightbar_ratio);
    config.max_lightbar_ratio = read_number(node, "max_lightbar_ratio", config.max_lightbar_ratio);
    config.min_lightbar_length = read_number(node, "min_lightbar_length", config.min_lightbar_length);
    config.min_armor_ratio = read_number(node, "min_armor_ratio", config.min_armor_ratio);
    config.max_armor_ratio = read_number(node, "max_armor_ratio", config.max_armor_ratio);
    config.max_angle_error = read_number(node, "max_angle_error", config.max_angle_error);
    config.max_side_ratio = read_number(node, "max_side_ratio", config.max_side_ratio);
    config.min_classifier_confidence =
        read_number(node, "min_classifier_confidence", config.min_classifier_confidence);
    config.yaw_offset = read_number(node, "yaw_offset", config.yaw_offset) * CV_PI / 180.0;
    config.pitch_offset = read_number(node, "pitch_offset", config.pitch_offset) * CV_PI / 180.0;
    config.camera_delay = read_number(node, "camera_delay", config.camera_delay);
    config.gravity = read_number(node, "gravity", config.gravity);
    config.classifier_model = node["classifier_model"].value_or(std::string());

    if (const auto array = node["camera_matrix"].as_array(); array != nullptr && array->size() >= 9) {
        config.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        for (int i = 0; i < 9; ++i) {
            config.camera_matrix.at<double>(i / 3, i % 3) =
                (*array)[static_cast<std::size_t>(i)].value_or(0.0);
        }
    }
    if (const auto array = node["distortion"].as_array(); array != nullptr && array->size() >= 5) {
        config.distortion = cv::Mat::zeros(1, 5, CV_64F);
        for (int i = 0; i < 5; ++i) {
            config.distortion.at<double>(0, i) =
                (*array)[static_cast<std::size_t>(i)].value_or(0.0);
        }
    }
    return config;
}

std::list<Armor> AutoAim::detect_armors(const cv::Mat& image) const {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::Mat binary;
    cv::threshold(gray, binary, config_.threshold, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    std::vector<Lightbar> lightbars;
    for (const auto& contour : contours) {
        if (contour.size() < 5) {
            continue;
        }
        const auto rotated = cv::minAreaRect(contour);
        const auto box = rotated.boundingRect();
        const float length = static_cast<float>(std::max(box.width, box.height));
        const float width = static_cast<float>(std::max(1, std::min(box.width, box.height)));
        const float ratio = length / width;
        if (length < config_.min_lightbar_length || ratio < config_.min_lightbar_ratio ||
            ratio > config_.max_lightbar_ratio) {
            continue;
        }

        std::array<cv::Point2f, 4> corners{};
        rotated.points(corners.data());
        std::sort(corners.begin(), corners.end(), [](const auto& a, const auto& b) {
            return a.y < b.y;
        });
        const auto top = midpoint(corners[0], corners[1]);
        const auto bottom = midpoint(corners[2], corners[3]);
        const float angle = std::atan2(bottom.y - top.y, bottom.x - top.x);
        if (std::abs(std::abs(angle) - CV_PI / 2.0) > config_.max_angle_error * CV_PI / 180.0) {
            continue;
        }

        double red_sum = 0.0;
        double blue_sum = 0.0;
        for (const auto& point : contour) {
            const auto& pixel = image.at<cv::Vec3b>(point);
            blue_sum += pixel[0];
            red_sum += pixel[2];
        }
        const auto color = red_sum > blue_sum ? ArmorColor::red : ArmorColor::blue;
        lightbars.push_back({rotated.center, top, bottom, length, width, color});
    }

    std::sort(lightbars.begin(), lightbars.end(), [](const auto& a, const auto& b) {
        return a.center.x < b.center.x;
    });

    std::list<Armor> armors;
    for (std::size_t i = 0; i < lightbars.size(); ++i) {
        for (std::size_t j = i + 1; j < lightbars.size(); ++j) {
            const auto& left = lightbars[i];
            const auto& right = lightbars[j];
            if (left.color != right.color) {
                continue;
            }
            const float distance = cv::norm(right.center - left.center);
            const float ratio = distance / std::max(left.length, right.length);
            const float side_ratio =
                std::max(left.length, right.length) / std::max(1.0F, std::min(left.length, right.length));
            const float pair_angle = std::abs(std::atan2(
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
            armor.confidence = 1.0F;
            armors.push_back(std::move(armor));
        }
    }
    return armors;
}

std::optional<Armor> AutoAim::classify_and_select(
    std::list<Armor>& armors, const cv::Mat& image) {
    armors.remove_if([this](const Armor& armor) {
        return armor.color != config_.enemy_color && config_.enemy_color != ArmorColor::unknown;
    });
    if (armors.empty()) {
        return std::nullopt;
    }

    for (auto& armor : armors) {
        const auto bounds = cv::boundingRect(armor.points) & cv::Rect(0, 0, image.cols, image.rows);
        if (!classifier_.empty() && bounds.area() > 0) {
            cv::Mat input;
            cv::resize(image(bounds), input, cv::Size(32, 32));
            classifier_.setInput(cv::dnn::blobFromImage(input, 1.0 / 255.0));
            const cv::Mat output = classifier_.forward();
            cv::Point label;
            double confidence = 0.0;
            cv::minMaxLoc(output.reshape(1, 1), nullptr, &confidence, nullptr, &label);
            armor.confidence = static_cast<float>(confidence);
            armor.name = parse_name(label.x);
            armor.type = parse_type(armor.name, cv::norm(armor.points[1] - armor.points[0]) /
                                                   std::max(1.0, cv::norm(armor.points[0] - armor.points[3])));
            if (confidence < config_.min_classifier_confidence) {
                armor.name = ArmorName::unknown;
            }
        } else {
            armor.type = parse_type(armor.name, 2.0);
        }
    }

    armors.sort([](const Armor& a, const Armor& b) {
        return cv::norm(a.center) < cv::norm(b.center);
    });
    return armors.front();
}

void AutoAim::solve_pose(Armor& armor) const {
    if (config_.camera_matrix.empty() || armor.points.size() != 4) {
        return;
    }
    const double width = armor.type == ArmorType::big ? BIG_ARMOR_WIDTH : SMALL_ARMOR_WIDTH;
    const std::vector<cv::Point3f> object_points{
        {0.0F, static_cast<float>(width / 2.0), static_cast<float>(LIGHTBAR_LENGTH / 2.0)},
        {0.0F, static_cast<float>(-width / 2.0), static_cast<float>(LIGHTBAR_LENGTH / 2.0)},
        {0.0F, static_cast<float>(-width / 2.0), static_cast<float>(-LIGHTBAR_LENGTH / 2.0)},
        {0.0F, static_cast<float>(width / 2.0), static_cast<float>(-LIGHTBAR_LENGTH / 2.0)}};
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    if (!cv::solvePnP(
            object_points, armor.points, config_.camera_matrix, config_.distortion, rvec, tvec,
            false, cv::SOLVEPNP_IPPE)) {
        return;
    }
    armor.position_gimbal = Eigen::Vector3d{tvec[0], tvec[1], tvec[2]};
    armor.position_world = gimbal_to_world_ * armor.position_gimbal;
    cv::Mat rotation;
    cv::Rodrigues(rvec, rotation);
    armor.yaw = std::atan2(rotation.at<double>(1, 0), rotation.at<double>(0, 0));
}

void AutoAim::update_target(
    const Armor& armor, std::chrono::steady_clock::time_point timestamp) {
    const double dt = last_timestamp_.time_since_epoch().count() == 0
                          ? 0.0
                          : std::chrono::duration<double>(timestamp - last_timestamp_).count();
    if (!target_.valid || dt <= 0.0 || dt > 0.1) {
        target_.center_world = armor.position_world;
        target_.velocity_world.setZero();
        target_.yaw = armor.yaw;
        target_.yaw_velocity = 0.0;
    } else {
        const auto predicted = target_.center_world + target_.velocity_world * dt;
        target_.velocity_world = (armor.position_world - predicted) / dt;
        target_.center_world = armor.position_world;
        const double yaw_delta = wrap_angle(armor.yaw - target_.yaw);
        target_.yaw_velocity = 0.7 * target_.yaw_velocity + 0.3 * yaw_delta / dt;
        target_.yaw = armor.yaw;
    }
    target_.name = armor.name;
    target_.type = armor.type;
    target_.armor_count = armor.name == ArmorName::outpost || armor.name == ArmorName::base ? 3 : 4;
    target_.valid = true;
}

AimCommand AutoAim::aim(const Target& target, double bullet_speed, double now_delay) const {
    AimCommand command;
    if (!target.valid) {
        return command;
    }
    if (bullet_speed < 10.0 || bullet_speed > 35.0) {
        bullet_speed = 23.0;
    }

    double flight_time = 0.0;
    Eigen::Vector4d aim_point = target.predict_armor(now_delay, 0);
    for (int i = 0; i < 6; ++i) {
        aim_point = target.predict_armor(now_delay + flight_time, 0);
        flight_time = std::max(0.0, std::hypot(aim_point.x(), aim_point.y()) / bullet_speed);
    }

    const double horizontal = std::hypot(aim_point.x(), aim_point.y());
    const double pitch = std::atan2(
        aim_point.z() + 0.5 * config_.gravity * flight_time * flight_time, horizontal);
    command.valid = std::isfinite(pitch);
    command.yaw = wrap_angle(std::atan2(aim_point.y(), aim_point.x()) + config_.yaw_offset);
    command.pitch = -pitch - config_.pitch_offset;
    command.flight_time = flight_time;
    command.fire = command.valid && std::abs(target.yaw_velocity) < 20.0;
    return command;
}

double AutoAim::wrap_angle(double angle) {
    while (angle > CV_PI) angle -= 2.0 * CV_PI;
    while (angle < -CV_PI) angle += 2.0 * CV_PI;
    return angle;
}

ArmorColor AutoAim::parse_color(const std::string& value) {
    if (value == "red") return ArmorColor::red;
    if (value == "blue") return ArmorColor::blue;
    return ArmorColor::unknown;
}

ArmorName AutoAim::parse_name(int class_id) {
    switch (class_id) {
        case 0: return ArmorName::one;
        case 1: return ArmorName::two;
        case 2: return ArmorName::three;
        case 3: return ArmorName::four;
        case 4: return ArmorName::five;
        case 5: return ArmorName::sentry;
        case 6: return ArmorName::outpost;
        case 7: return ArmorName::base;
        default: return ArmorName::unknown;
    }
}

ArmorType AutoAim::parse_type(ArmorName name, double ratio) {
    if (name == ArmorName::one || name == ArmorName::base || ratio > 3.0) {
        return ArmorType::big;
    }
    return ArmorType::small;
}

}  // namespace app::auto_aim
