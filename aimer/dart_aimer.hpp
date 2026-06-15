/**
 * @file dart_aimer.hpp
 * @brief 飞镖瞄准器 — 抛物线弹道补偿 + 指令生成
 *
 * 飞镖弹道与步兵直线弹道的核心差异:
 *   - 飞镖初速低 (~10-15 m/s)，弹道呈明显抛物线
 *   - 飞行时间较长 (~0.3-0.8s)，重力/空气阻力影响显著
 *   - 需要抬高 pitch 补偿重力下坠
 *
 * 瞄准模型:
 *   - 已知目标距离 d (从 solver 解算的世界系位置获取)
 *   - 已知飞镖初速 v0 (从配置/yaml 获取)
 *   - 计算: 所需 pitch 角 = atan( (v0² - sqrt(v0⁴ - g·(g·d² + 2·h·v0²))) / (g·d) )
 *       (抛物线真空弹道解析解)
 *   - yaw: 直接指向目标水平方向
 *
 * 输出 io::Command (yaw, pitch, control, shoot)
 */

#ifndef DART_AUTO_AIM__DART_AIMER_HPP
#define DART_AUTO_AIM__DART_AIMER_HPP

#include <Eigen/Dense>
#include <string>

#include "io/command.hpp"
#include "tasks/dart_auto_aim/common/dart_types.hpp"

namespace dart_auto_aim
{

/**
 * @class DartAimer
 * @brief 飞镖瞄准器 — 弹道解算 + 串口指令生成
 */
class DartAimer
{
public:
  /** @brief 从 YAML 配置构造 */
  explicit DartAimer(const std::string & config_path);

  /**
   * @brief 生成瞄准指令
   * @param target        跟踪目标 (含 world 位姿)
   * @param is_tracking   是否正在跟踪
   * @param bullet_speed  飞镖初速 [m/s]
   * @param image_width   图像宽度 (归一化用)
   * @return io::Command  云台控制指令
   */
  io::Command aim(
    const DartTarget & target,
    bool is_tracking,
    double bullet_speed,
    int image_width = 1440) const;

  /**
   * @brief 简易瞄准 — 仅用归一化中心偏移 (兼容 wust_vision 接口)
   * @param center_norm_x 归一化水平中心偏移 [-1, 1]
   * @param is_tracking   是否正在跟踪
   * @return io::Command
   */
  io::Command aim_simple(float center_norm_x, bool is_tracking) const;

  // ── 参数 ──
  double dart_speed() const { return dart_speed_; }

private:
  // ── 弹道参数 ──
  double dart_speed_  = 12.0;   ///< 飞镖初速 [m/s]
  double gravity_     = 9.81;   ///< 重力加速度 [m/s²]
  double drag_coeff_  = 0.01;   ///< 空气阻力系数 (简化一阶)

  // ── 瞄准偏置 ──
  double yaw_offset_   = 0.0;   ///< yaw 偏置 [rad]
  double pitch_offset_ = 0.0;   ///< pitch 偏置 [rad]

  // ── 控制参数 ──
  double pitch_scale_  = 1.0;   ///< pitch 缩放系数 (调整灵敏度)
  double yaw_scale_    = 0.5;   ///< yaw 缩放系数

  // ── 发射阈值 ──
  double fire_tolerance_x_ = 0.05;  ///< 归一化中心容差 (0.05 = 图像宽度的 2.5%)
  double fire_tolerance_y_ = 0.05;  ///< 归一化中心容差

  /** @brief 抛物线 pitch 角解析解 (真空，忽略空气阻力) */
  double parabolic_pitch(double distance_h, double height_diff, double v0) const;

  /** @brief 带空气阻力的 pitch 角 (数值求解/查表) */
  double drag_pitch(double distance, double height_diff, double v0) const;
};

}  // namespace dart_auto_aim

#endif  // DART_AUTO_AIM__DART_AIMER_HPP
