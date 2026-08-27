#include "auto_aim.hpp"

#include "tools/logger.hpp"
#include "tools/tomlpp.hpp"

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace app::auto_aim {
namespace {

constexpr const char* MODULE = "AUTO_AIM";

template <typename Node>
double read_number(const Node& node, const char* key, double fallback) {
    return node[key].value_or(fallback);
}

std::vector<double> read_array(const auto& node, const char* key) {
    std::vector<double> values;
    const auto array = node[key].as_array();
    if (array == nullptr) return values;
    values.reserve(array->size());
    for (const auto& value : *array) values.push_back(value.value_or(0.0));
    return values;
}

Eigen::Matrix3d matrix3_from_array(const std::vector<double>& values) {
    Eigen::Matrix3d matrix{
        {0.0, 0.0, 1.0},
        {1.0, 0.0, 0.0},
        {0.0, -1.0, 0.0}};
    if (values.size() < 9) return matrix;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            matrix(row, column) = values[static_cast<std::size_t>(row * 3 + column)];
        }
    }
    return matrix;
}

Eigen::Vector3d vector3_from_array(const std::vector<double>& values) {
    if (values.size() < 3) return Eigen::Vector3d::Zero();
    return {values[0], values[1], values[2]};
}

ArmorColor parse_color(const std::string& value) {
    if (value == "red") return ArmorColor::red;
    if (value == "blue") return ArmorColor::blue;
    return ArmorColor::unknown;
}

}  // namespace

AutoAim::AutoAim(const std::string& config_path)
    : detector_([&config_path] {
          const auto table = toml::parse_file(config_path);
          const auto node = table["auto_aim"];
          DetectorConfig config;
          config.enemy_color =
              parse_color(node["enemy_color"].value_or(std::string("blue")));
          config.threshold = read_number(node, "threshold", config.threshold);
          config.min_lightbar_ratio =
              read_number(node, "min_lightbar_ratio", config.min_lightbar_ratio);
          config.max_lightbar_ratio =
              read_number(node, "max_lightbar_ratio", config.max_lightbar_ratio);
          config.min_lightbar_length =
              read_number(node, "min_lightbar_length", config.min_lightbar_length);
          config.min_armor_ratio =
              read_number(node, "min_armor_ratio", config.min_armor_ratio);
          config.max_armor_ratio =
              read_number(node, "max_armor_ratio", config.max_armor_ratio);
          config.max_angle_error =
              read_number(node, "max_angle_error", config.max_angle_error);
          config.max_side_ratio =
              read_number(node, "max_side_ratio", config.max_side_ratio);
          return config;
      }()),
      classifier_([&config_path] {
          const auto table = toml::parse_file(config_path);
          const auto node = table["auto_aim"];
          ClassifierConfig config;
          config.model_path = node["classifier_model"].value_or(std::string());
          config.min_confidence =
              read_number(node, "min_classifier_confidence", config.min_confidence);
          return config;
      }()),
      solver_([&config_path] {
          const auto table = toml::parse_file(config_path);
          const auto node = table["auto_aim"];
          SolverConfig config;
          const auto camera_matrix = read_array(node, "camera_matrix");
          const auto distortion = read_array(node, "distortion");
          if (camera_matrix.size() >= 9) {
              config.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
              for (int index = 0; index < 9; ++index) {
                  config.camera_matrix.at<double>(index / 3, index % 3) =
                      camera_matrix[static_cast<std::size_t>(index)];
              }
          }
          if (distortion.size() >= 5) {
              config.distortion = cv::Mat::zeros(1, 5, CV_64F);
              for (int index = 0; index < 5; ++index) {
                  config.distortion.at<double>(0, index) =
                      distortion[static_cast<std::size_t>(index)];
              }
          }
          config.camera_to_gimbal =
              matrix3_from_array(read_array(node, "camera_to_gimbal"));
          config.camera_to_gimbal_translation =
              vector3_from_array(read_array(node, "camera_to_gimbal_translation"));
          config.optimize_yaw = node["optimize_yaw"].value_or(true);
          return config;
      }()),
      tracker_([&config_path] {
          const auto table = toml::parse_file(config_path);
          const auto node = table["auto_aim"];
          TrackerConfig config;
          config.min_detect_count = static_cast<int>(
              read_number(node, "min_detect_count", config.min_detect_count));
          config.max_temp_lost_count = static_cast<int>(
              read_number(node, "max_temp_lost_count", config.max_temp_lost_count));
          config.outpost_max_temp_lost_count = static_cast<int>(
              read_number(node, "outpost_max_temp_lost_count",
                          config.outpost_max_temp_lost_count));
          config.normal_radius = read_number(node, "normal_radius", config.normal_radius);
          config.outpost_radius = read_number(node, "outpost_radius", config.outpost_radius);
          config.base_radius = read_number(node, "base_radius", config.base_radius);
          return config;
      }()),
      aimer_([&config_path] {
          const auto table = toml::parse_file(config_path);
          const auto node = table["auto_aim"];
          AimerConfig config;
          config.yaw_offset = read_number(node, "yaw_offset", 0.0) * CV_PI / 180.0;
          config.pitch_offset = read_number(node, "pitch_offset", 0.0) * CV_PI / 180.0;
          config.gravity = read_number(node, "gravity", config.gravity);
          config.camera_delay = read_number(node, "camera_delay", config.camera_delay);
          config.high_speed_delay =
              read_number(node, "high_speed_delay", config.high_speed_delay);
          config.low_speed_delay =
              read_number(node, "low_speed_delay", config.low_speed_delay);
          config.decision_speed =
              read_number(node, "decision_speed", config.decision_speed);
          return config;
      }()) {
    const auto table = toml::parse_file(config_path);
    camera_delay_ = read_number(table["auto_aim"], "camera_delay", camera_delay_);
    LOG_INFO(MODULE, "armor auto aim initialized");
}

AutoAimResult AutoAim::process(
    const cv::Mat& image,
    std::chrono::steady_clock::time_point timestamp,
    const Eigen::Matrix3d& gimbal_to_world,
    double bullet_speed) {
    AutoAimResult result;
    if (image.empty()) {
        result.target = tracker_.track(result.armors, timestamp);
        state_ = tracker_.state();
        if (result.target.has_value()) {
            result.command = aimer_.aim(*result.target, bullet_speed, camera_delay_);
        }
        return result;
    }

    solver_.set_gimbal_to_world(gimbal_to_world);
    result.armors = detector_.detect(image);
    classifier_.classify(result.armors, image);

    result.armors.remove_if([](const Armor& armor) {
        return armor.color == ArmorColor::unknown;
    });
    for (auto iterator = result.armors.begin(); iterator != result.armors.end();) {
        if (!solver_.solve(*iterator)) {
            iterator = result.armors.erase(iterator);
        } else {
            ++iterator;
        }
    }

    auto tracked_armors = result.armors;
    result.target = tracker_.track(tracked_armors, timestamp);
    state_ = tracker_.state();
    if (result.target.has_value()) {
        result.command = aimer_.aim(*result.target, bullet_speed, camera_delay_);
    }
    return result;
}

const std::string& AutoAim::state() const {
    return state_;
}

}  // namespace app::auto_aim
