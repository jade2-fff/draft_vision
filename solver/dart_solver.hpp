/**
 * @file dart_solver.hpp
 * @brief 飞镖靶 PnP 解算器 — 像素→相机→世界 坐标变换链
 *
 * 飞镖靶是圆形平面靶，用外接矩形的 4 个角点做 PnP 解算。
 * 世界模型: 靶面 14cm×14cm 正方形 (half=0.07m)，竖直悬挂。
 *
 * 变换链: pixel → PnP(camera) → hand-eye(gimbal) → imu(world)
 *
 * 与步兵 armor solver 的核心区别:
 *   - 飞镖靶是静态靶面，不需 optimize_yaw
 *   - 靶面尺寸固定 (0.14m×0.14m)
 *   - 用 bbox 四角直接做 PnP，不依赖灯条匹配
 */

#ifndef DART_AUTO_AIM__DART_SOLVER_HPP
#define DART_AUTO_AIM__DART_SOLVER_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <string>

#include "tasks/dart_auto_aim/common/dart_types.hpp"

namespace dart_auto_aim
{

/**
 * @class DartSolver
 * @brief 飞镖靶 PnP 解算器
 *
 * 使用方式:
 * @code
 *   DartSolver solver("configs/dart.yaml");
 *   solver.set_R_gimbal2world(imu_quaternion);
 *   for (auto & target : detections) solver.solve(target);
 * @endcode
 */
class DartSolver
{
public:
  /** @brief 从 YAML 加载标定参数 */
  explicit DartSolver(const std::string & config_path);

  /** @return 当前云台→世界旋转矩阵 */
  Eigen::Matrix3d R_gimbal2world() const { return R_gimbal2world_; }

  /** @brief 每帧更新 IMU 姿态 */
  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  /**
   * @brief 对单个飞镖靶做 PnP 解算
   * @param[in,out] target  输入 bbox/角点; 输出 position_camera/ypr_in_world 等
   * @return PnP 是否成功
   */
  bool solve(DartTarget & target) const;

  /** @brief 获取相机内参 (调试用) */
  const cv::Mat & camera_matrix() const { return camera_matrix_; }
  const cv::Mat & distort_coeffs() const { return distort_coeffs_; }

private:
  cv::Mat camera_matrix_;       ///< 3×3 内参
  cv::Mat distort_coeffs_;      ///< 畸变系数 [k1,k2,p1,p2,k3]

  // ── 手眼标定 ──
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;

  Eigen::Matrix3d R_gimbal2world_;  ///< 每帧由 IMU 更新

  // ── 靶面模型参数 ──
  double target_half_size_ = 0.07;  ///< 靶面半边长 [m] (14cm 靶面)
};

}  // namespace dart_auto_aim

#endif  // DART_AUTO_AIM__DART_SOLVER_HPP
