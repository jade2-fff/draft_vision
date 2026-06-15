/**
 * @file dart_tracker.hpp
 * @brief 飞镖靶 EKF 跟踪器 — 图像空间 8 状态卡尔曼滤波
 *
 * 状态向量 (X_N=8):
 *   [cx, vx, cy, vy, w, vw, h, vh]
 *   - cx, cy: bbox 中心像素坐标
 *   - w, h:   bbox 宽高 [px]
 *   - vx, vy, vw, vh: 对应速度
 *
 * 观测向量 (Z_N=4):
 *   [cx, cy, w, h]  — 检测器输出的 bbox
 *
 * 飞镖靶是静止悬挂的圆形靶面，在图像平面做匀速运动假设。
 * 跟踪器维护状态机: LOST → DETECTING → TRACKING → TEMP_LOST → LOST
 */

#ifndef DART_AUTO_AIM__DART_TRACKER_HPP
#define DART_AUTO_AIM__DART_TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <vector>

#include "tasks/dart_auto_aim/common/dart_types.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace dart_auto_aim
{

// ── EKF 维度常量 ──
constexpr int DART_X_N = 8;   ///< 状态维度: cx, vx, cy, vy, w, vw, h, vh
constexpr int DART_Z_N = 4;   ///< 观测维度: cx, cy, w, h

/**
 * @class DartTracker
 * @brief 飞镖靶 EKF 跟踪器
 *
 * 使用方式:
 * @code
 *   DartTracker tracker("configs/dart.yaml");
 *   auto target = tracker.track(detections, timestamp);
 *   if (tracker.is_tracking()) { ... }
 * @endcode
 */
class DartTracker
{
public:
  /** @brief 跟踪器状态 */
  enum class State {
    LOST,         ///< 未检测到目标
    DETECTING,    ///< 检测到但未达跟踪阈值
    TRACKING,     ///< 稳定跟踪
    TEMP_LOST     ///< 短暂丢失
  };

  /** @brief 从 YAML 配置构造 */
  explicit DartTracker(const std::string & config_path);

  /**
   * @brief 主跟踪入口 — 输入检测结果，输出跟踪状态
   * @param detections 当前帧检测结果
   * @param timestamp  时间戳
   * @return 跟踪目标 (未跟踪时 confidence<0)
   */
  DartTarget track(
    const std::vector<DartTarget> & detections,
    std::chrono::steady_clock::time_point timestamp);

  // ── 状态查询 ──
  State  state()        const { return state_; }
  bool   is_tracking()  const { return state_ == State::TRACKING || state_ == State::TEMP_LOST; }
  bool   is_lost()      const { return state_ == State::LOST; }
  std::string state_str() const;

  // ── EKF 调试接口 ──
  const tools::ExtendedKalmanFilter & ekf() const { return ekf_; }
  Eigen::VectorXd ekf_x() const { return ekf_.x; }

  /** @brief 目标中心 (像素坐标) */
  cv::Point2f center() const {
    return cv::Point2f(static_cast<float>(ekf_.x[0]), static_cast<float>(ekf_.x[2]));
  }

private:
  // ── 状态机 ──
  State state_       = State::LOST;
  State prev_state_  = State::LOST;

  // ── 计数器 ──
  int detect_count_      = 0;
  int temp_lost_count_   = 0;
  int total_lost_count_  = 0;

  // ── 阈值 ──
  int    min_detect_count_   = 5;     ///< 进入 TRACKING 所需连续检测帧数
  int    max_temp_lost_count_= 15;    ///< TEMP_LOST 超时帧数
  double lost_timeout_       = 0.5;   ///< LOST 判定时间 [s]

  // ── EKF ──
  tools::ExtendedKalmanFilter ekf_;
  double process_noise_xy_ = 100.0;   ///< cx/cy 过程噪声 q
  double process_noise_wh_ = 100.0;   ///< w/h 过程噪声 q
  double measure_noise_xy_ = 0.05;    ///< cx/cy 观测噪声 r
  double measure_noise_wh_ = 0.05;    ///< w/h 观测噪声 r

  // ── 时间 ──
  std::chrono::steady_clock::time_point last_t_;

  // ── 当前目标 ──
  DartTarget tracked_target_;

  // ── 内部方法 ──
  void state_machine(bool found);
  void init_ekf(const DartTarget & t);
  bool associate(const std::vector<DartTarget> & detections, DartTarget & matched) const;
  void update_ekf(const DartTarget & t);
};

}  // namespace dart_auto_aim

#endif  // DART_AUTO_AIM__DART_TRACKER_HPP
