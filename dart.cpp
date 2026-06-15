/**
 * @file dart.cpp
 * @brief 飞镖自瞄主程序
 *
 * 管线: 相机 → 检测 → 跟踪 → 归一化偏移 → 串口
 * 下位机负责所有补偿，上位机只输出目标偏移量。
 */
#include <fmt/core.h>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/dart_auto_aim/detector/dart_detector.hpp"
#include "tasks/dart_auto_aim/solver/dart_solver.hpp"
#include "tasks/dart_auto_aim/tracker/dart_tracker.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

using namespace std::chrono;

int main(int argc, char * argv[]) {
  std::string config_path = argc > 1 ? argv[1] : "configs/dart.yaml";

  tools::Exiter exiter;
  io::Gimbal gimbal(config_path);
  io::Camera camera(config_path);

  dart_auto_aim::DartDetector detector(config_path);
  dart_auto_aim::DartSolver   solver(config_path);
  dart_auto_aim::DartTracker  tracker(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  int frame_id = 0;

  tools::logger()->info("[Dart] Starting...");

  while (!exiter.exit()) {
    frame_id++;
    camera.read(img, t);

    auto q = gimbal.q(t - 2ms);
    solver.set_R_gimbal2world(q);

    // 检测 → 跟踪
    auto detections = detector.detect(img, t, frame_id);
    auto tracked = tracker.track(detections, t);

    // 归一化 yaw 偏移
    float cx_norm = tracker.is_tracking() ? tracked.center_norm_x() : 0.f;
    bool fire = tracker.is_tracking() && std::abs(cx_norm) < 0.05f;

    // 串口: 只发归一化偏移 (兼容区域赛 draft 协议)
    float send_val = fire ? cx_norm : 0.f;
    gimbal.send(fire, fire, send_val, 0, 0, 0, 0, 0);

    // 调试显示
    tools::draw_text(img,
      fmt::format("[Dart] {} | det={} | cx={:.3f}",
        tracker.state_str(), detections.size(), cx_norm),
      {10, 30}, {255, 255, 255});

    if (tracker.is_tracking()) {
      cv::circle(img, tracked.center, 3, cv::Scalar(0, 0, 255), -1);
      cv::circle(img, tracked.center, (int)tracked.radius, cv::Scalar(0, 255, 0), 2);
    }

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("[Dart]", img);
    if (cv::waitKey(1) == 'q') break;
  }

  tools::logger()->info("[Dart] Exit.");
  return 0;
}
