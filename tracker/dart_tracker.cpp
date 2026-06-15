#include "tasks/dart_auto_aim/tracker/dart_tracker.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"

namespace dart_auto_aim
{

// ── 构造 ────────────────────────────────────────────────

DartTracker::DartTracker(const std::string & config_path)
{
  YAML::Node config = YAML::LoadFile(config_path);

  if (config["dart_tracker"]) {
    auto tc = config["dart_tracker"];

    if (tc["target"]) {
      auto tgt = tc["target"];
      measure_noise_xy_  = tgt["xy_r"].as<double>(measure_noise_xy_);
      measure_noise_wh_  = tgt["wh_r"].as<double>(measure_noise_wh_);
      process_noise_xy_  = tgt["q_xy"].as<double>(process_noise_xy_);
      process_noise_wh_  = tgt["q_wh"].as<double>(process_noise_wh_);
    }

    min_detect_count_    = tc["tracking_thres"].as<int>(min_detect_count_);
    max_temp_lost_count_ = tc["max_temp_lost_count"].as<int>(max_temp_lost_count_);
    lost_timeout_        = tc["lost_time_thres"].as<double>(lost_timeout_);
  }

  tools::logger()->info("[DartTracker] Init: det_thres={} temp_lost={} timeout={:.2f}s",
    min_detect_count_, max_temp_lost_count_, lost_timeout_);
}

// ── 主跟踪入口 ──────────────────────────────────────────

DartTarget DartTracker::track(
  const std::vector<DartTarget> & detections,
  std::chrono::steady_clock::time_point timestamp)
{
  bool found = false;
  DartTarget matched;

  if (state_ != State::LOST && !detections.empty()) {
    // 关联最近邻
    found = associate(detections, matched);
  } else if (state_ == State::LOST && !detections.empty()) {
    // 首次检测或重捕: 取最高置信度
    double best_conf = -1.0;
    for (const auto & d : detections) {
      if (d.confidence > best_conf) {
        best_conf = d.confidence;
        matched = d;
        found = true;
      }
    }
  }

  // ── 状态机 ──
  state_machine(found);

  // ── EKF 处理 ──
  if (state_ == State::DETECTING && found) {
    detect_count_++;
    if (detect_count_ >= min_detect_count_) {
      init_ekf(matched);
      state_ = State::TRACKING;
      detect_count_ = 0;
      tools::logger()->info("[DartTracker] EKF initialized — entering TRACKING");
    }
    tracked_target_ = matched;
  } else if ((state_ == State::TRACKING || state_ == State::TEMP_LOST) && found) {
    update_ekf(matched);
    tracked_target_ = matched;
  } else if (state_ == State::TRACKING || state_ == State::TEMP_LOST) {
    // 无观测 → 仅预测
    double dt = std::chrono::duration<double>(timestamp - last_t_).count();
    // 匀速预测
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(DART_X_N, DART_X_N);
    F(0, 1) = dt;
    F(2, 3) = dt;
    F(4, 5) = dt;
    F(6, 7) = dt;

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(DART_X_N, DART_X_N);
    Q(1, 1) = process_noise_xy_ * dt;
    Q(3, 3) = process_noise_xy_ * dt;
    Q(5, 5) = process_noise_wh_ * dt;
    Q(7, 7) = process_noise_wh_ * dt;

    ekf_.predict(F, Q);
    // 用预测值更新 tracked_target_
    tracked_target_.center = cv::Point2f(static_cast<float>(ekf_.x[0]), static_cast<float>(ekf_.x[2]));
    tracked_target_.bbox = cv::Rect(
      static_cast<int>(ekf_.x[0] - ekf_.x[4] / 2),
      static_cast<int>(ekf_.x[2] - ekf_.x[6] / 2),
      static_cast<int>(ekf_.x[4]),
      static_cast<int>(ekf_.x[6]));
  }

  if (found || is_tracking()) {
    last_t_ = timestamp;
    tracked_target_.timestamp = timestamp;
  }

  return tracked_target_;
}

// ── 状态机 ──────────────────────────────────────────────

void DartTracker::state_machine(bool found)
{
  prev_state_ = state_;

  switch (state_) {
    case State::LOST:
      if (found) state_ = State::DETECTING;
      break;

    case State::DETECTING:
      if (!found) {
        state_ = State::LOST;
        detect_count_ = 0;
      }
      break;

    case State::TRACKING:
      if (found) {
        temp_lost_count_ = 0;
      } else {
        temp_lost_count_++;
        if (temp_lost_count_ > max_temp_lost_count_) {
          state_ = State::LOST;
          temp_lost_count_ = 0;
          detect_count_ = 0;
          total_lost_count_++;
          tools::logger()->info("[DartTracker] Target LOST (timeout {}/{} frames)",
            temp_lost_count_, max_temp_lost_count_);
        } else {
          state_ = State::TEMP_LOST;
        }
      }
      break;

    case State::TEMP_LOST:
      if (found) {
        temp_lost_count_ = 0;
        state_ = State::TRACKING;
        tools::logger()->info("[DartTracker] Target re-acquired");
      } else {
        temp_lost_count_++;
        if (temp_lost_count_ > max_temp_lost_count_) {
          state_ = State::LOST;
          temp_lost_count_ = 0;
          detect_count_ = 0;
          total_lost_count_++;
        }
      }
      break;
  }
}

// ── EKF 初始化 ──────────────────────────────────────────

void DartTracker::init_ekf(const DartTarget & t)
{
  // 状态: [cx, vx=0, cy, vy=0, w, vw=0, h, vh=0]
  Eigen::VectorXd x0(DART_X_N);
  x0 << t.center.x, 0.0,
        t.center.y, 0.0,
        static_cast<double>(t.bbox.width),  0.0,
        static_cast<double>(t.bbox.height), 0.0;

  Eigen::VectorXd P0_diag(DART_X_N);
  P0_diag << 10.0, 1000.0,   // cx, vx
             10.0, 1000.0,   // cy, vy
             10.0, 1000.0,   // w, vw
             10.0, 1000.0;   // h, vh
  Eigen::MatrixXd P0 = P0_diag.asDiagonal();

  ekf_ = tools::ExtendedKalmanFilter(x0, P0);
}

// ── 最近邻关联 ──────────────────────────────────────────

bool DartTracker::associate(
  const std::vector<DartTarget> & detections,
  DartTarget & matched) const
{
  if (detections.empty()) return false;

  // 预测位置做关联
  cv::Point2f pred_center(
    static_cast<float>(ekf_.x[0]),
    static_cast<float>(ekf_.x[2]));

  double best_dist = 1e10;
  const DartTarget * best = nullptr;

  for (const auto & d : detections) {
    double dx = d.center.x - pred_center.x;
    double dy = d.center.y - pred_center.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    // 最大关联距离: bbox 对角线的一半
    double max_dist = std::sqrt(ekf_.x[4] * ekf_.x[4] + ekf_.x[6] * ekf_.x[6]) * 0.75;
    if (dist < best_dist && dist < max_dist) {
      best_dist = dist;
      best = &d;
    }
  }

  if (best) {
    matched = *best;
    return true;
  }

  return false;
}

// ── EKF 更新 ────────────────────────────────────────────

void DartTracker::update_ekf(const DartTarget & t)
{
  double dt = std::chrono::duration<double>(t.timestamp - last_t_).count();
  if (dt <= 0.0) dt = 0.033;  // 防止零/负 dt
  if (dt > 1.0) dt = 0.1;     // 防止大间隔

  // ── 预测 (匀速模型) ──
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(DART_X_N, DART_X_N);
  F(0, 1) = dt;
  F(2, 3) = dt;
  F(4, 5) = dt;
  F(6, 7) = dt;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(DART_X_N, DART_X_N);
  Q(1, 1) = process_noise_xy_ * dt;
  Q(3, 3) = process_noise_xy_ * dt;
  Q(5, 5) = process_noise_wh_ * dt;
  Q(7, 7) = process_noise_wh_ * dt;

  ekf_.predict(F, Q);

  // ── 观测 ──
  Eigen::VectorXd z(DART_Z_N);
  z << t.center.x, t.center.y,
       static_cast<double>(t.bbox.width), static_cast<double>(t.bbox.height);

  // 观测矩阵 (线性: z = H * x)
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(DART_Z_N, DART_X_N);
  H(0, 0) = 1.0;  // cx
  H(1, 2) = 1.0;  // cy
  H(2, 4) = 1.0;  // w
  H(3, 6) = 1.0;  // h

  // 观测噪声 (动态: 目标越远噪声越大)
  double dist_factor = 1.0 + std::abs(ekf_.x[4] - 100.0) / 200.0;
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(DART_Z_N, DART_Z_N);
  R(0, 0) = measure_noise_xy_ * dist_factor;
  R(1, 1) = measure_noise_xy_ * dist_factor;
  R(2, 2) = measure_noise_wh_ * dist_factor;
  R(3, 3) = measure_noise_wh_ * dist_factor;

  ekf_.update(z, H, R);
}

// ── 状态名 ──────────────────────────────────────────────

std::string DartTracker::state_str() const
{
  switch (state_) {
    case State::LOST:      return "LOST";
    case State::DETECTING: return "DETECTING";
    case State::TRACKING:  return "TRACKING";
    case State::TEMP_LOST: return "TEMP_LOST";
    default:               return "UNKNOWN";
  }
}

}  // namespace dart_auto_aim
