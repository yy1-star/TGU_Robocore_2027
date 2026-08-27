#include <cassert>
#include <chrono>

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include "app/auto_aim/auto_aim.hpp"

int main(int argc, char** argv) {
    assert(argc == 2);

    cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::rectangle(image, {260, 180}, {272, 260}, {255, 0, 0}, cv::FILLED);
    cv::rectangle(image, {368, 180}, {380, 260}, {255, 0, 0}, cv::FILLED);

    app::auto_aim::AutoAim auto_aim(argv[1]);
    const auto result = auto_aim.process(
        image,
        std::chrono::steady_clock::now(),
        Eigen::Matrix3d::Identity(),
        23.0);

    assert(result.armors.size() == 1);
    assert(result.target.has_value());
    assert(result.command.valid);
    return 0;
}
