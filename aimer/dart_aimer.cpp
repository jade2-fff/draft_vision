#include "tasks/dart_auto_aim/aimer/dart_aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>

#include "tools/logger.hpp"

namespace dart_auto_aim
{

DartAimer::DartAimer(const std::string & config_path)
{
  YAML::Node config = YAML::LoadFile(config_path);

  if (config["dart_aimer"]) {
    auto ac = config["dart_aimer"];

    dart_speed_   = ac["dart_speed"].as<double>(dart_speed_);
    gravity_      = ac["gravity"].as<double>(gravity_);
    drag_coeff_   = ac["drag_coeff"].as<double>(drag_coeff_);

    yaw_offset_   = ac["yaw_offset"].as<double>(yaw_offset_) * M_PI / 180.0;
    pitch_offset_ = ac["pitch_offset"].as<double>(pitch_offset_) * M_PI / 180.0;

    pitch_scale_  = ac["pitch_scale"].as<double>(pitch_scale_);
    yaw_scale_    = ac["yaw_scale"].as<double>(yaw_scale_);

    fire_tolerance_x_ = ac["fire_tolerance_x"].as<double>(fire_tolerance_x_);
    fire_tolerance_y_ = ac["fire_tolerance_y"].as<double>(fire_tolerance_y_);
  }

  tools::logger()->info("[DartAimer] Init: v0={:.1f} m/s g={:.2f} drag={:.4f}",
    dart_speed_, gravity_, drag_coeff_);
}

io::Command DartAimer::aim(
  const DartTarget & target,
  bool is_tracking,
  double bullet_speed,
  int image_width) const
{
  io::Command cmd;
  cmd.control = false;
  cmd.shoot   = false;
  cmd.yaw     = 0.0;
  cmd.pitch   = 0.0;

  if (!is_tracking) return cmd;
  if (!target.solved) {
    // 未解算 → 使用归一化中心偏移做简易跟踪
    float cx_norm = target.center_norm_x();
    cmd.control = true;
    cmd.yaw   = -cx_norm * yaw_scale_;
    cmd.pitch = 0.0;
    // 不射击 (无距离信息)
    return cmd;
  }

  // ── 从世界系位置计算瞄准角 ──
  const auto & p = target.position_world;

  double distance_xy = std::sqrt(p.x() * p.x() + p.y() * p.y());
  double height_diff = p.z();  // 目标相对高度

  // 水平 yaw
  double target_yaw = std::atan2(p.y(), p.x());

  // 弹道 pitch (使用实际弹速或默认值)
  double v0 = (bullet_speed > 1.0) ? bullet_speed : dart_speed_;
  double target_pitch = parabolic_pitch(distance_xy, height_diff, v0);

  // 带空气阻力修正
  if (drag_coeff_ > 1e-6) {
    target_pitch += drag_pitch(distance_xy, height_diff, v0);
  }

  // 应用偏置
  cmd.yaw   = target_yaw   + yaw_offset_;
  cmd.pitch = target_pitch + pitch_offset_;
  cmd.control = true;

  // ── 射击判定: 目标在画面中心附近且距离合理 ──
  float cx_norm = target.center_norm_x();
  bool on_target =
    std::abs(cx_norm) < fire_tolerance_x_ &&
    distance_xy > 0.5 && distance_xy < 8.0;  // 距离范围 0.5~8m

  cmd.shoot = on_target;

  return cmd;
}

io::Command DartAimer::aim_simple(float center_norm_x, bool is_tracking) const
{
  io::Command cmd;
  if (!is_tracking) {
    cmd.control = false;
    cmd.shoot   = false;
    cmd.yaw     = 0.0;
    cmd.pitch   = 0.0;
    return cmd;
  }

  // 简易 P 控制器: 归一化偏移 → yaw 指令
  cmd.control = true;
  cmd.yaw     = -center_norm_x * yaw_scale_;
  cmd.pitch   = 0.0;

  // 射击判定
  cmd.shoot = std::abs(center_norm_x) < fire_tolerance_x_;

  return cmd;
}

double DartAimer::parabolic_pitch(double d, double h, double v0) const
{
  // 抛物线真空弹道:
  //   θ = arctan( (v0² ± sqrt(v0⁴ - g·(g·d² + 2·h·v0²))) / (g·d) )
  //   '+' 为高抛角，飞镖取 '-' (低弹道)

  double g = gravity_;
  double v02 = v0 * v0;
  double v04 = v02 * v02;
  double discriminant = v04 - g * (g * d * d + 2.0 * h * v02);

  if (discriminant < 0.0) {
    // 超出射程，取 45° 极限
    return M_PI / 4.0;
  }

  double sqrt_disc = std::sqrt(discriminant);
  double theta = std::atan2(v02 - sqrt_disc, g * d);

  return theta;
}

double DartAimer::drag_pitch(double distance, double height_diff, double v0) const
{
  // 一阶空气阻力修正: Δθ ≈ k * d * g / v0²
  // 简化模型: 阻力使弹道更低，需额外抬高
  double g = gravity_;
  double base = drag_coeff_ * distance * g / (v0 * v0);
  return base;  // 正值=额外抬高角
}

}  // namespace dart_auto_aim
