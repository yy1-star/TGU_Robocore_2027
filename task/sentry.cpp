//
// Created by tgu on 2026/5/20.
//

#include "tools/logger.hpp"
#include "io/hikrobot/hikrobot.hpp"

#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <thread>

#include "app/auto_aim/auto_aim.hpp"

// 配置文件路径
const auto CONFIG_PATH = "../config/sentry.toml";

// 运行状态
std::atomic_bool running = true;

// logger
tools::LoggerConfig cfg{
    .level = tools::LogLevel::Debug, .enable_console = true, .enable_file = false, .file_path = "logs.txt"
};
static constexpr const char *MODULE = "SENTRY_MAIN";

// 相机
struct Frame {
    cv::Mat image;
    uint64_t timestamp_ns;
};

std::atomic<std::shared_ptr<Frame> > frame{nullptr};

// 相机采集线程
void camera_thread() {
    io::Hikrobot camera(CONFIG_PATH);
    while (running) {
        if (!camera.init()) {
            LOG_ERROR(MODULE, "camera init failed");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        if (!camera.start()) {
            LOG_ERROR(MODULE, "camera start failed");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        cv::Mat image{};
        uint64_t timestamp_ns = 0;
        uint32_t error_count = 0;

        while (camera.is_running() && running) {
            if (error_count >= 20) {
                LOG_ERROR(MODULE, "camera disconnected");
                break;
            }

            if (!camera.grab(image, timestamp_ns)) {
                error_count++;
                continue;
            }

            frame.store(std::make_shared<Frame>(Frame{.image = image.clone(), .timestamp_ns = timestamp_ns}));
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
    }
    LOG_INFO(MODULE, "Camera thread stopped");
}

// 自瞄处理线程。当前仓库还没有云台通信接口，因此这里只生成并记录瞄准命令。
void auto_aim_thread() {
    app::auto_aim::AutoAim auto_aim(CONFIG_PATH);
    std::string previous_state;

    while (running) {
        const auto frame_ = frame.load();
        if (!frame_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }

        const auto result = auto_aim.process(
            frame_->image,
            std::chrono::steady_clock::now(),
            Eigen::Matrix3d::Identity(),
            23.0);
        const auto state = auto_aim.state();
        if (state != previous_state) {
            LOG_INFO(MODULE, "auto aim state: {}", state);
            previous_state = state;
        }
        if (result.command.valid) {
            LOG_DEBUG(
                MODULE,
                "auto aim command yaw={:.4f}, pitch={:.4f}, fire={}",
                result.command.yaw,
                result.command.pitch,
                result.command.fire);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

int main() {
    tools::Logger::instance().init(cfg);

    std::jthread camera(camera_thread);
    std::jthread auto_aim_worker(auto_aim_thread);

    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}
